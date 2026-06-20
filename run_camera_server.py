#!/usr/bin/env python3
import os
import re
import socket
import struct
import subprocess
import sys
import time
import numpy as np
import threading
# ==========================================
# CONFIGURATION
# ==========================================
HOST = "0.0.0.0"
THERMAL_PORT = 5002

# Thermal Camera Specs
WIDTH = 96
HEIGHT = 176
THERM_W = 96
THERM_H = 96
MAGIC = b"TFRM"
HEADER_FMT = "<4sIHH"
FRAME_BYTES = WIDTH * HEIGHT * 2

# Global list to keep track of background processes for clean shutdown
active_processes = []

# ==========================================
# 1. HARDWARE DISCOVERY ENGINE
# ==========================================
def get_camera_mapping():
    """Scans Linux video nodes to identify cameras by their hardware signatures."""
    base_dir = "/sys/class/video4linux"
    raw_devices = {}

    if not os.path.exists(base_dir):
        print("[Discovery] ❌ Error: /sys/class/video4linux not found.")
        return {}

    for node in os.listdir(base_dir):
        if not node.startswith("video"):
            continue
        try:
            with open(os.path.join(base_dir, node, "name"), "r") as f:
                dev_name = f.read().strip()
            
            # THE FIX: Extract the true physical USB topology (e.g., "1-2" or "1-6")
            sys_path = os.readlink(os.path.join(base_dir, node))
            port_match = re.search(r'/([0-9]+-[0-9\.]+):', sys_path)
            hardware_id = port_match.group(1) if port_match else f"unknown_{node}"
            
            video_index = int(node.replace("video", ""))
            
            if dev_name not in raw_devices:
                raw_devices[dev_name] = {}
            if hardware_id not in raw_devices[dev_name]:
                raw_devices[dev_name][hardware_id] = []
                
            raw_devices[dev_name][hardware_id].append(video_index)
        except Exception:
            continue

    mapping = {"thermal": None, "ai_realsense": None, "nav_left": None, "nav_right": None}

    # Thermal Camera (UVC Camera wrapper)
    for dev_name, ports in raw_devices.items():
        if "UVC Camera" in dev_name or "Camera:" in dev_name:
            all_nodes = sorted([n for p in ports.values() for n in p])
            if all_nodes: mapping["thermal"] = f"/dev/video{all_nodes[0]}"

    # Intel RealSense (Select 5th index)
    for dev_name, ports in raw_devices.items():
        if "RealSense" in dev_name:
            all_nodes = sorted([n for p in ports.values() for n in p])
            if len(all_nodes) >= 5: mapping["ai_realsense"] = f"/dev/video{all_nodes[4]}"
            elif all_nodes: mapping["ai_realsense"] = f"/dev/video{all_nodes[0]}"

    # DJI Osmo Action 4 Cameras (Separated by physical USB port)
    osmo_data = []
    for dev_name, ports in raw_devices.items():
        if "Osmo" in dev_name:
            for hardware_id, nodes in ports.items():
                if nodes: osmo_data.append((hardware_id, sorted(nodes)[0]))
    
    # Sort by physical port ID so Left/Right stays consistent across reboots!
    osmo_data.sort(key=lambda x: x[0])
    if len(osmo_data) >= 1: mapping["nav_left"] = f"/dev/video{osmo_data[0][1]}"
    if len(osmo_data) >= 2: mapping["nav_right"] = f"/dev/video{osmo_data[1][1]}"

    return mapping


# ==========================================
# 2. STREAM ORCHESTRATOR
# ==========================================
def launch_pipeline(name: str, command: list):
    print(f"[{name}] Starting pipeline...")
    proc = subprocess.Popen(command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    active_processes.append(proc)
    return proc

def start_video_streams(mapping):
    print("\n--- Starting GStreamer Video Pipelines ---")
    
    if mapping["nav_left"]:
        cmd = [
            "gst-launch-1.0", "v4l2src", f"device={mapping['nav_left']}", "do-timestamp=true",
            "!", "image/jpeg,width=1280,height=720,framerate=30/1",
            "!", "rtpjpegpay", "mtu=1200",
            "!", "udpsink", "host=127.0.0.1", "port=5004", "sync=false", "async=false"
        ]
        launch_pipeline("Nav Left (5004)", cmd)
    else: print("⚠️ Nav Left (Osmo 1) not found.")

    if mapping["nav_right"]:
        cmd = [
            "gst-launch-1.0", "v4l2src", f"device={mapping['nav_right']}", "do-timestamp=true",
            "!", "image/jpeg,width=1280,height=720,framerate=30/1",
            "!", "rtpjpegpay", "mtu=1200",
            "!", "udpsink", "host=127.0.0.1", "port=5006", "sync=false", "async=false"
        ]
        launch_pipeline("Nav Right (5006)", cmd)
    else: print("⚠️ Nav Right (Osmo 2) not found.")

    if mapping["ai_realsense"]:
        cmd = [
            "gst-launch-1.0", "v4l2src", f"device={mapping['ai_realsense']}", "do-timestamp=true",
            "!", "video/x-raw,width=1280,height=720,framerate=30/1",
            "!", "jpegenc",
            "!", "rtpjpegpay", "mtu=1200",
            "!", "udpsink", "host=127.0.0.1", "port=5000", "sync=false", "async=false"
        ]
        launch_pipeline("AI Camera (5000)", cmd)
    else: print("⚠️ AI Camera (RealSense) not found.")


# ==========================================
# 3. THERMAL STREAM SERVER
# ==========================================
def start_ffmpeg(thermal_device):
    cmd = [
        "ffmpeg", "-f", "v4l2", "-input_format", "yuyv422",
        "-video_size", f"{WIDTH}x{HEIGHT}", "-i", thermal_device,
        "-f", "rawvideo", "-pix_fmt", "yuyv422", "-"
    ]
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=FRAME_BYTES * 4)
    active_processes.append(proc)
    return proc

def read_temperature_matrix(proc):
    raw = proc.stdout.read(FRAME_BYTES)
    if len(raw) != FRAME_BYTES: return None
    u16 = np.frombuffer(raw, dtype=np.uint16).reshape(HEIGHT, WIDTH)
    thermal = u16[:THERM_H, :THERM_W]
    temps = (thermal.astype(np.float32) - 4607.03) / 27.03
    return temps

def serve_thermal_client(conn, thermal_device):
    print("[Thermal] UI Client connected!")
    proc = start_ffmpeg(thermal_device)
    frame_num = 0

    try:
        while True:
            temps = read_temperature_matrix(proc)
            if temps is None:
                print("[Thermal] Lost FFmpeg frame.")
                break

            header = struct.pack(HEADER_FMT, MAGIC, frame_num, THERM_W, THERM_H)
            payload = temps.astype("<f4").tobytes()
            conn.sendall(header + payload)
            frame_num += 1

    except (BrokenPipeError, ConnectionResetError):
        pass
    finally:
        try:
            proc.kill()
            active_processes.remove(proc)
        except: pass
        conn.close()
        print("[Thermal] UI Client disconnected.")

def run_thermal_server():
    """Runs in a background thread and grabs the newest device path on connection."""
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, THERMAL_PORT))
    server.listen(1)
    
    print(f"\n[Thermal] Serving matrix stream on port {THERMAL_PORT}...")
    while True:
        conn, addr = server.accept()
        # Dynamically fetch the current mapping right as the UI connects!
        current_mapping = get_camera_mapping()
        thermal_dev = current_mapping.get("thermal")
        
        if thermal_dev:
            serve_thermal_client(conn, thermal_dev)
        else:
            print("[Thermal] ⚠️ No thermal camera connected right now.")
            conn.close()

# ==========================================\n# MAIN EXECUTION
# ==========================================\n
# ==========================================
# MAIN EXECUTION
# ==========================================
def main():
    print("=== DYNAMIC CAMERA STREAM ORCHESTRATOR ===")
    
    # 1. Start the thermal server in its own daemon thread
    thermal_thread = threading.Thread(target=run_thermal_server, daemon=True)
    thermal_thread.start()

    # 2. Initial hardware scan and launch
    current_mapping = get_camera_mapping()
    for cam, path in current_mapping.items():
        print(f"  🎯 {cam.upper():<15} -> {path or 'NOT FOUND'}")
    start_video_streams(current_mapping)

    # 3. WATCHDOG LOOP
    try:
        while True:
            time.sleep(3)
            
            # Scan the Linux hardware again
            new_mapping = get_camera_mapping()
            
            # Did a camera get unplugged, or did a frozen process finally die?
            hardware_changed = (new_mapping != current_mapping)
            crashed = any(proc.poll() is not None for proc in active_processes)
            
            if crashed or hardware_changed:
                if hardware_changed:
                    print("\n[Watchdog] 🔌 USB Disconnect/Reconnect detected!")
                else:
                    print("\n[Watchdog] ⚠️ Camera process crashed!")
                    
                print("Killing frozen feeds and Rebooting streams...")
                
                # Forcefully kill all background streams
                for proc in active_processes:
                    try:
                        proc.kill()  # kill() is stronger than terminate() for frozen pipelines
                    except:
                        pass
                active_processes.clear()
                
                # Give Linux 2 seconds to finish registering the new USBs
                time.sleep(2) 
                
                # Update our map and start the feeds again!
                current_mapping = get_camera_mapping()
                for cam, path in current_mapping.items():
                    print(f"  🎯 {cam.upper():<15} -> {path or 'NOT FOUND'}")
                start_video_streams(current_mapping)
                
    except KeyboardInterrupt:
        print("\n\n[System] Ctrl+C detected. Shutting down all streams...")
        for proc in active_processes:
            try:
                proc.kill()
            except:
                pass
        print("[System] Goodbye!")
        sys.exit(0)

if __name__ == "__main__":
    main()