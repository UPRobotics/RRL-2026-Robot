# Latency Improvements — RRL 2026 Robot

End-to-end command latency path:

```
Operator joystick
  → joy_node (/dev/input/js0, USB HID ~1 ms)
  → /joy topic
  → joystick_node
  → /joystick/* topics  ←── WiFi DDS transport (main variable)
  → body_node / arm_node
  → drive timer fires (up to 10 ms / 20 ms later)
  → VESC.set_rpm() over UART (115200 baud, ~1 ms)
  → Motor
```

---

## 1. Disable WiFi power saving — do this first

**Effort:** 30 seconds  
**Gain:** Eliminates 10–50 ms latency spikes (the most common complaint on mobile robots)

WiFi adapters sleep between packets by default. When a packet arrives during sleep the AP buffers it until the next beacon interval (typically 100 ms). Disabling power save forces the adapter to stay awake.

```bash
# Run on BOTH the robot (Jetson) and the ground station
sudo iwconfig wlan0 power off
sudo iw dev wlan0 set power_save off

# Verify
iwconfig wlan0 | grep "Power Management"
# Should show: Power Management:off
```

To make it permanent (survives reboot), create `/etc/NetworkManager/conf.d/wifi-powersave-off.conf`:

```ini
[connection]
wifi.powersave = 2
```

On the Jetson specifically, also lock clocks to max:

```bash
sudo nvpmodel -m 0
sudo jetson_clocks
```

---

## 2. Switch DDS middleware to CycloneDDS

**Effort:** 5 minutes  
**Gain:** 2–4 ms less steady-state overhead vs FastDDS, better WiFi behavior out of the box

```bash
# Install on both machines
sudo apt install ros-humble-rmw-cyclonedds-cpp

# Add to .bashrc on both machines (or the launch environment)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

CycloneDDS unicast config — create `cyclonedds.xml` in the `fastdds/` folder:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain id="any">
    <General>
      <NetworkInterfaceAddress>AUTO</NetworkInterfaceAddress>
      <AllowMulticast>false</AllowMulticast>
      <MaxMessageSize>65500B</MaxMessageSize>
      <FragmentSize>4000B</FragmentSize>
    </General>
    <Discovery>
      <Peers>
        <Peer Address="ROBOT_IP"/>
        <Peer Address="STATION_IP"/>
      </Peers>
      <ParticipantIndex>auto</ParticipantIndex>
    </Discovery>
    <Internal>
      <Watermarks>
        <WhcHigh>500kB</WhcHigh>
      </Watermarks>
    </Internal>
  </Domain>
</CycloneDDS>
```

```bash
# Add to .bashrc alongside RMW_IMPLEMENTATION
export CYCLONEDDS_URI=file:///path/to/fastdds/cyclonedds.xml
```

---

## 3. Increase OS UDP socket buffers

**Effort:** 2 minutes  
**Gain:** Reduces packet drops under burst traffic, eliminates retransmit delays on RELIABLE topics

```bash
# Apply immediately (both machines)
sudo sysctl -w net.core.rmem_max=26214400
sudo sysctl -w net.core.wmem_max=26214400
sudo sysctl -w net.core.rmem_default=26214400
sudo sysctl -w net.core.wmem_default=26214400

# Make permanent — add to /etc/sysctl.d/99-ros2-udp.conf
net.core.rmem_max     = 26214400
net.core.wmem_max     = 26214400
net.core.rmem_default = 26214400
net.core.wmem_default = 26214400
```

---

## 4. Dedicated AP + 5 GHz channel

**Effort:** 15 minutes (router config)  
**Gain:** 5–20 ms reduction in variance, eliminates interference-driven retries

- Use a dedicated WiFi AP only for the robot — no phones, laptops, or other devices sharing it
- Use **5 GHz** (802.11ac/ax) instead of 2.4 GHz — fewer competing networks, lower retry rate
- Set a **fixed channel** on the AP (e.g. channel 36 or 149) — auto-channel causes mid-run channel scans that spike latency
- Set channel width to **80 MHz** for throughput headroom

---

## 5. Consolidate joystick topics into one message

**Effort:** Medium (new message type + update control_pkg, body_node, arm_node)  
**Gain:** 4× fewer DDS messages per control cycle, eliminates torn reads between axes

Currently `joystick_node` publishes 4 separate `Float32` topics:

```
/joystick/left_x   ← separate UDP packet
/joystick/left_y   ← separate UDP packet
/joystick/right_x  ← separate UDP packet
/joystick/right_y  ← separate UDP packet
```

Each is serialized, sent, and received independently. The tank drive mixes `left_y + left_x` — if those two arrive in different ROS callbacks there is a one-cycle inconsistency where one axis is already updated and the other is not.

Replace with a single custom message published once per `/joy` callback:

```
# robot_msgs/msg/JoystickAxes.msg
float32 left_x
float32 left_y
float32 right_x
float32 right_y
```

One UDP packet, one callback, all axes always consistent.

---

## 6. Reduce arm drive timer period: 20 ms → 10 ms

**Effort:** 2 lines of code  
**Gain:** Halves worst-case arm command lag (from 20 ms to 10 ms)

Body drive timers already fire at 10 ms (100 Hz). Arm drive timers fire at 20 ms (50 Hz).
In `arm.cpp`, change all six timer declarations:

```cpp
// Before
timer_hip_ = this->create_wall_timer(20ms, ...);

// After
timer_hip_ = this->create_wall_timer(10ms, ...);
```

---

## Priority Order

| # | Fix | Effort | Impact |
|---|-----|--------|--------|
| 1 | Disable WiFi power saving | 30 s | Highest — eliminates spikes |
| 2 | CycloneDDS | 5 min | High — lower steady-state latency |
| 3 | UDP socket buffers | 2 min | Medium — reduces drops |
| 4 | Dedicated AP / 5 GHz | 15 min | Medium — reduces variance |
| 5 | Arm timer 20ms → 10ms | 1 min | Low — reduces command lag |
| 6 | Consolidate joystick topics | Hours | Low-medium — cleaner architecture |
