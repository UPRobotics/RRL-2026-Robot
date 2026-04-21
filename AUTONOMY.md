# Autonomy Implementation Guide — Ekbalam

Flat-ground autonomous navigation for a tracked skid-steer robot using the onboard Livox MID-360 LiDAR.

**Target stack:**
```
PointCloud → pointcloud_to_laserscan → 2D LaserScan
           → slam_toolbox (2D SLAM: map + pose)
           → Nav2 (path planning + obstacle avoidance)
           → /cmd_vel → skid-steer controller → L track / R track
```

**ROS 2 distro:** Humble  
**Hardware:** Livox MID-360 (LiDAR + integrated IMU), Jetson Orin, 2× VESC tracks

---

## Architecture Overview

```
[Livox MID-360]
    ├─ /livox/lidar  (PointCloud2)
    │       └─ pointcloud_to_laserscan → /scan  (LaserScan, frame: base_link)
    │       └─ fastlio_pkg            → /Odometry  (LiDAR-inertial odometry)
    └─ /livox/imu   (Imu)
            └─ fastlio_pkg (internal)

[TF tree]
    map → odom → base_link → lidar_link
    (slam_toolbox publishes map→odom correction)
    (odom_relay publishes odom→base_link from FAST-LIO)
    (robot_state_publisher publishes base_link→lidar_link from URDF)

[Nav2]
    /map + /scan + TF → costmap → planner → /cmd_vel (Twist)

[Skid-steer controller node]  ← NEW
    /cmd_vel (Twist) → /control/input (ControlInput) → body_node → VESCs
```

---

## Pre-Work: Fix Broken Hardware State

Before writing any code, verify the physical robot is drivable.

**Left track is disabled in config.json** — `motors[1].enabled = false` (Track Izquierdo). Re-enable it and confirm both tracks respond:

```bash
# Edit src/robot_pkg/config/config.json
# Set motors[1].enabled to true

# Rebuild and test
colcon build --packages-select robot_pkg
source install/setup.bash
ros2 launch robot_pkg body.launch.py
# Drive manually and confirm both tracks move
```

---

## Phase 1 — TF Tree and Robot Description

**Why first:** Every downstream component (pointcloud_to_laserscan, slam_toolbox, Nav2 costmap) requires a valid TF tree rooted at `base_link`. Nothing works without this.

### 1a. Create a minimal URDF

Create `src/robot_pkg/urdf/ekbalam.urdf.xacro` with:
- `base_link` — robot body origin (center between tracks, at ground level)
- `lidar_link` — child of `base_link`, offset = measured XYZ position of the MID-360 on the robot chassis

The FAST-LIO extrinsic in `src/fastlio_pkg/config/mid360.yaml` gives the IMU-to-LiDAR offset as a starting point:
```yaml
mapping.extrinsic_T: [-0.011, -0.02329, 0.04412]
```
Measure the actual LiDAR mount position on Ekbalam and set it here.

### 1b. Launch robot_state_publisher

Add to `src/robot_pkg/launch/body.launch.py` (or a new `autonomy.launch.py`):
```python
Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'robot_description': urdf_content, 'use_sim_time': False}]
)
```

This publishes the static `base_link → lidar_link` TF.

### 1c. Remap FAST-LIO frame names

FAST-LIO publishes with frames `camera_init` (world) and `body` (robot). Nav2 expects `odom` and `base_link`. Write a small relay node (`src/robot_pkg/src/odom_relay.cpp`) that:
1. Subscribes to `/Odometry` (`nav_msgs/Odometry`, parent=`camera_init`, child=`body`)
2. Republishes it on `/odom` with `header.frame_id = "odom"`, `child_frame_id = "base_link"`
3. Broadcasts the `odom → base_link` TF

This reuses FAST-LIO's high-quality LiDAR-inertial odometry instead of writing a wheel odometry node — tracked robots slip significantly, making encoder-only odometry unreliable.

**Resulting TF tree after Phase 1:**
```
odom → base_link → lidar_link
```
(`map → odom` is added by slam_toolbox in Phase 3)

---

## Phase 2 — PointCloud to LaserScan

**Install:**
```bash
sudo apt install ros-humble-pointcloud-to-laserscan
```
Add to `install_ubuntu.sh` and `install_jetson_orin.sh`.

**Configure:** Create `src/robot_pkg/config/laserscan.yaml`:
```yaml
pointcloud_to_laserscan_node:
  ros__parameters:
    target_frame: base_link     # transform cloud into this frame first
    transform_tolerance: 0.01
    min_height: -0.1            # meters below base_link to include
    max_height: 0.5             # meters above base_link to include
    angle_min: -3.14159         # -180 deg
    angle_max:  3.14159         #  180 deg
    angle_increment: 0.00436    # ~0.25 deg resolution
    scan_time: 0.1
    range_min: 0.3
    range_max: 30.0
    use_inf: true
```

Tune `min_height`/`max_height` to slice the point cloud at a height that captures obstacles without hitting the ground plane. The MID-360 is a full-hemisphere scanner — the height filter is critical.

**Add to launch file:**
```python
Node(
    package='pointcloud_to_laserscan',
    executable='pointcloud_to_laserscan_node',
    remappings=[('cloud_in', '/livox/lidar'), ('scan', '/scan')],
    parameters=[laserscan_params]
)
```

**Verify:** `ros2 run rviz2 rviz2` — add a LaserScan display on `/scan`, confirm the 2D ring of points correctly outlines obstacles around the robot.

---

## Phase 3 — 2D SLAM (slam_toolbox)

**Install:**
```bash
sudo apt install ros-humble-slam-toolbox
```
Add to install scripts.

**Mode:** Use `online_async` mode (real-time, no saved map required to start).

**Create** `src/robot_pkg/config/slam_toolbox.yaml`:
```yaml
slam_toolbox:
  ros__parameters:
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    odom_frame: odom
    map_frame: map
    base_frame: base_link
    scan_topic: /scan
    mode: mapping
    use_sim_time: false
    # Scan matcher tuning — adjust for Livox point density
    minimum_travel_distance: 0.2
    minimum_travel_heading: 0.1
    map_update_interval: 5.0
    resolution: 0.05            # 5 cm map resolution
    max_laser_range: 20.0
```

slam_toolbox subscribes to `/scan` and the `odom → base_link` TF, and publishes:
- `/map` (`nav_msgs/OccupancyGrid`)
- TF correction: `map → odom`

**Verify:** In RViz add Map display on `/map`. Drive manually, confirm the map builds correctly.

---

## Phase 4 — Skid-Steer Controller Node

**This is the only new C++ node to write.**

Nav2 outputs `/cmd_vel` (`geometry_msgs/Twist` with `linear.x` and `angular.z`). `body_node` consumes `robot_msgs/ControlInput`. A bridge node is needed.

**Create** `src/robot_pkg/src/cmd_vel_to_control.cpp`:

Key logic:
```cpp
// Subscribe: /cmd_vel (geometry_msgs/Twist)
// Publish:   /control/input (robot_msgs/ControlInput)

void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg) {
    const float v = msg->linear.x;
    const float w = msg->angular.z;
    // W = track separation in meters (measure on Ekbalam)
    const float W = 0.40f;
    float left  = v + (W / 2.0f) * w;
    float right = v - (W / 2.0f) * w;
    // Normalize to [-1, 1] based on max speed
    // max_speed corresponds to rpm_limit in config.json (1500 RPM for tracks)
    const float max_speed = MAX_TRACK_SPEED_MPS;
    left  = std::clamp(left  / max_speed, -1.0f, 1.0f);
    right = std::clamp(right / max_speed, -1.0f, 1.0f);

    robot_msgs::msg::ControlInput out{};
    out.mode   = 0;          // MOVEMENT mode
    out.left_y  = (left + right) / 2.0f;   // reconstitute tank mix inputs
    out.left_x  = (left - right) / 2.0f;   // body_node applies: L=y+x, R=y-x
    pub_->publish(out);
}
```

> The `body_node` drive formula is `left = left_y + left_x`, `right = left_y - left_x`. Solving for `left_y` and `left_x` given desired left/right track speeds gives the formulas above.

**Also add** a `/robot/mode` publisher that holds mode=0 (MOVEMENT) while autonomy is active, so `body_node` doesn't reject drive commands.

**Track separation W:** Measure the distance between the center lines of the left and right tracks on Ekbalam. Update `W` in the node or make it a ROS parameter.

**Max speed calibration:** At rpm_limit=1500, measure actual linear speed (m/s) to set `MAX_TRACK_SPEED_MPS`.

---

## Phase 5 — Nav2

**Install:**
```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```
Add to install scripts.

**Create** `src/robot_pkg/config/nav2_params.yaml`. Key sections to configure for a tracked robot:

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      desired_linear_vel: 0.3       # m/s — start conservative
      lookahead_dist: 0.6
      min_lookahead_dist: 0.3
      max_lookahead_dist: 0.9
      rotate_to_heading_angular_vel: 0.5
      use_rotate_to_heading: true   # tracked robots can rotate in place

local_costmap:
  local_costmap:
    ros__parameters:
      footprint: "[[0.35, 0.25], [0.35, -0.25], [-0.35, -0.25], [-0.35, 0.25]]"
      # Replace with Ekbalam's actual footprint (meters, centered on base_link)

global_costmap:
  global_costmap:
    ros__parameters:
      footprint: "[[0.35, 0.25], [0.35, -0.25], [-0.35, -0.25], [-0.35, 0.25]]"
```

Measure Ekbalam's physical footprint (length × width) and update the `footprint` polygon.

**Verify:** Send a `geometry_msgs/PoseStamped` goal via RViz's "2D Goal Pose" tool. Watch the robot navigate.

---

## Phase 6 — Integrated Launch File

Create `src/robot_pkg/launch/autonomy.launch.py` that starts the full stack:

| Node | Package | Purpose |
|------|---------|---------|
| `livox_lidar_publisher` | `livox_ros_driver2` | LiDAR + IMU data |
| `fastlio_mapping` | `fastlio_pkg` | LiDAR-inertial odometry |
| `odom_relay_node` | `robot_pkg` | Remap FAST-LIO frames → odom/base_link |
| `robot_state_publisher` | `robot_state_publisher` | URDF TF (base_link→lidar_link) |
| `pointcloud_to_laserscan_node` | `pointcloud_to_laserscan` | 3D→2D scan |
| `async_slam_toolbox_node` | `slam_toolbox` | 2D SLAM + map |
| `nav2_bringup` | `nav2_bringup` | Full Nav2 stack |
| `cmd_vel_to_control_node` | `robot_pkg` | /cmd_vel → /control/input |
| `body_node` | `robot_pkg` | Track motor control |

---

## Phase 7 — Install Script Updates

After each phase, add the new apt packages to both install scripts:

```bash
# Phase 2
sudo apt install -y ros-humble-pointcloud-to-laserscan

# Phase 3
sudo apt install -y ros-humble-slam-toolbox

# Phase 5
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup

# New build dependency (robot_state_publisher)
sudo apt install -y ros-humble-robot-state-publisher ros-humble-xacro
```

---

## Key Parameters to Measure on the Physical Robot

These must be measured on Ekbalam before meaningful navigation is possible:

| Parameter | Where used | How to measure |
|-----------|-----------|----------------|
| Track separation `W` (m) | cmd_vel_to_control, Nav2 params | Measure center-to-center of left and right tracks |
| Robot footprint (m) | Nav2 costmap | Measure physical length and width of chassis |
| LiDAR mount offset XYZ (m) | URDF `lidar_link` | Measure from base_link origin to MID-360 center |
| Max linear speed at rpm_limit (m/s) | cmd_vel_to_control | Drive at full speed, measure distance/time |

---

## Verification Checkpoints

Run these in order — do not proceed to the next phase until each passes.

- [ ] **Phase 1:** `ros2 run tf2_tools view_frames` shows `odom → base_link → lidar_link`
- [ ] **Phase 2:** RViz LaserScan on `/scan` correctly outlines walls and obstacles
- [ ] **Phase 3:** RViz Map on `/map` builds a coherent 2D map while driving manually
- [ ] **Phase 4:** Publishing a test `/cmd_vel` (`ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}}"`) moves the robot forward
- [ ] **Phase 5:** Robot navigates to a 2D goal pose set in RViz without hitting obstacles

---

## Known Issues / Gotchas

- **Left track disabled:** `src/robot_pkg/config/config.json` index 1 (`Track Izquierdo`) has `"enabled": false`. Fix before any drive testing.
- **FAST-LIO lidar_type:** `mid360.yaml` sets `lidar_type: 1` (Livox CustomMsg). `livox_ros_driver2` must also be configured to publish CustomMsg format (`transfer_format` parameter), not PointCloud2. Verify they agree.
- **Livox IP configuration:** MID-360 is configured for `192.168.0.104` (LiDAR) and `192.168.0.254` (host) in `src/livox_ros_driver2/config/MID360_config.json`. Set the Jetson's ethernet interface to `192.168.0.254` before running.
- **FastDDS on WiFi:** Source `fastdds/setup_cyclone.sh` before launching if operating over WiFi. Multicast (default DDS) will not work.
- **Vosk model path:** `mic_pkg` launch hardcodes paths to `/home/chumbi/...`. Fix before running on a new machine (unrelated to autonomy but blocks `ros2 launch robot_pkg robot.launch.py`).
- **`pcd_save.map_file_path` in FAST-LIO config** is set to `/home/salvador/maps/` — change to a valid path on the Jetson before saving maps.
