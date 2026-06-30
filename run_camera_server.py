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
import shlex

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

# Global lists to keep track of processes distinctly
video_processes = []
thermal_processes = []
process_lock = threading.Lock()

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

    for dev_name, ports in raw_devices.items():
        if "UVC Camera" in dev_name or "Camera:" in dev_name:
            all_nodes = sorted([n for p in ports.values() for n in p])
            if all_nodes: mapping["thermal"] = f"/dev/video{all_nodes[0]}"

    for dev_name, ports in raw_devices.items():
        if "RealSense" in dev_name:
            all_nodes = sorted([n for p in ports.values() for n in p])
            if len(all_nodes) >= 5: mapping["ai_realsense"] = f"/dev/video{all_nodes[4]}"
            elif all_nodes: mapping["ai_realsense"] = f"/dev/video{all_nodes[0]}"

    osmo_data = []
    for dev_name, ports in raw_devices.items():
        if "Osmo" in dev_name:
            for hardware_id, nodes in ports.items():
                if nodes: osmo_data.append((hardware_id, sorted(nodes)[0]))
    
    osmo_data.sort(key=lambda x: x[0])
    if len(osmo_data) >= 1: mapping["nav_left"] = f"/dev/video{osmo_data[0][1]}"
    if len(osmo_data) >= 2: mapping["nav_right"] = f"/dev/video{osmo_data[1][1]}"

    return mapping


# ==========================================
# 2. STREAM ORCHESTRATOR
# ==========================================
def launch_pipeline(name: str, command: list):
    print(f"[{name}] Starting pipeline...")
    print(shlex.join(command))

    proc = subprocess.Popen(
        command,
        stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True
    )

    with process_lock:
        video_processes.append(proc)

    return proc

    # Give GStreamer a moment t
def start_video_streams(mapping):
    print("\n--- Starting Software H.264 GStreamer Video Pipelines ---")
    
    # ── NAV LEFT
    if mapping["nav_left"]:
        cmd = [
            "gst-launch-1.0", "v4l2src", f"device={mapping['nav_left']}", "do-timestamp=true",
            "!", "image/jpeg,width=1280,height=720,framerate=30/1",
            "!", "jpegdec",
            "!", "videoconvert",
            "!", "x264enc", "tune=zerolatency", "bitrate=1500", "speed-preset=ultrafast",
            "!", "rtph264pay", "config-interval=1", "pt=96",
            "!", "udpsink", "host=192.168.0.70", "port=5004", "sync=false", "async=false"
        ]
        launch_pipeline("Nav Left (5004 - H.264 720p)", cmd)
    else:
        print("⚠️ Nav Left (Osmo 1) not found.")

    # ── NAV RIGHT (Flipped 180 Degrees)
    if mapping["nav_right"]:
        cmd = [
            "gst-launch-1.0", "v4l2src", f"device={mapping['nav_right']}", "do-timestamp=true",
            "!", "image/jpeg,width=1280,height=720,framerate=30/1",
            "!", "jpegdec",
            "!", "videoconvert",
            "!", "videoflip", "method=rotate-180",
            "!", "x264enc", "tune=zerolatency", "bitrate=1500", "speed-preset=ultrafast",
            "!", "rtph264pay", "config-interval=1", "pt=96",
            "!", "udpsink", "host=192.168.0.70", "port=5006", "sync=false", "async=false"
        ]
        launch_pipeline("Nav Right (5006 - H.264 720p - Flipped 180)", cmd)
    else:
        print("⚠️ Nav Right (Osmo 2) not found.")

    # ── AI CAMERA (RealSense)
    if mapping["ai_realsense"]:
        cmd = [
            "gst-launch-1.0", "v4l2src", f"device={mapping['ai_realsense']}", "do-timestamp=true",
            "!", "video/x-raw,width=1280,height=720,framerate=30/1",
            "!", "videoconvert",
            "!", "x264enc", "tune=zerolatency", "bitrate=1500", "speed-preset=ultrafast",
            "!", "rtph264pay", "config-interval=1", "pt=96",
            "!", "udpsink", "host=192.168.0.70", "port=5000", "sync=false", "async=false"
        ]
        launch_pipeline("AI Camera (5000 - H.264 720p)", cmd)
    else:
        print("⚠️ AI Camera (RealSense) not found.")


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
    with process_lock:
        thermal_processes.append(proc)
    return proc

def read_temperature_matrix(proc):
    try:
        raw = proc.stdout.read(FRAME_BYTES)
        if len(raw) != FRAME_BYTES: return None
        u16 = np.frombuffer(raw, dtype=np.uint16).reshape(HEIGHT, WIDTH)
        thermal = u16[:THERM_H, :THERM_W]
        temps = (thermal.astype(np.float32) - 4607.03) / 27.03
        return temps
    except Exception:
        return None

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
            with process_lock:
                if proc in thermal_processes:
                    thermal_processes.remove(proc)
        except: pass
        conn.close()
        print("[Thermal] UI Client disconnected.")

def run_thermal_server():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, THERMAL_PORT))
    server.listen(1)
    
    print(f"[Thermal] Serving matrix stream on port {THERMAL_PORT}...")
    while True:
        try:
            conn, addr = server.accept()
            current_mapping = get_camera_mapping()
            thermal_dev = current_mapping.get("thermal")
            
            if thermal_dev:
                serve_thermal_client(conn, thermal_dev)
            else:
                print("[Thermal] ⚠️ No thermal camera connected right now.")
                conn.close()
        except Exception as e:
            print(f"[Thermal Server Error] {e}")
            time.sleep(1)


# ==========================================
# MAIN EXECUTION
# ==========================================
def main():
    print("=== DYNAMIC CAMERA STREAM ORCHESTRATOR ===")
    
    thermal_thread = threading.Thread(target=run_thermal_server, daemon=True)
    thermal_thread.start()

    current_mapping = get_camera_mapping()
    for cam, path in current_mapping.items():
        print(f"   🎯 {cam.upper():<15} -> {path or 'NOT FOUND'}")
    start_video_streams(current_mapping)

    try:
        while True:
            time.sleep(3)
            
            new_mapping = get_camera_mapping()
            hardware_changed = (new_mapping != current_mapping)
            
            with process_lock:
                crashed = any(proc.poll() is not None for proc in video_processes)
            
            if crashed or hardware_changed:
                if hardware_changed:
                    print("\n[Watchdog] 🔌 USB Disconnect/Reconnect detected!")
                else:
                    print("\n[Watchdog] ⚠️ Camera process crashed!")
                    
                print("Killing frozen video feeds and Rebooting streams...")
                
                with process_lock:
                    for proc in video_processes:
                        try:
                            proc.kill()
                            proc.wait()
                        except:
                            pass
                    video_processes.clear()
                
                time.sleep(2) 
                
                current_mapping = get_camera_mapping()
                for cam, path in current_mapping.items():
                    print(f"   🎯 {cam.upper():<15} -> {path or 'NOT FOUND'}")
                start_video_streams(current_mapping)
                
    except KeyboardInterrupt:
        print("\n\n[System] Ctrl+C detected. Shutting down all streams...")
        with process_lock:
            all_procs = video_processes + thermal_processes
            for proc in all_procs:
                try:
                    proc.kill()
                except:
                    pass
        print("[System] Goodbye!")
        sys.exit(0)

if __name__ == "__main__":
    main()
