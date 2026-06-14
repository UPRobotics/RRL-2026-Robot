#!/usr/bin/env python3
"""
ROS2 Magnetometer Receiver Node
Subscribes to magnetometer data and displays a GUI with current readings
and a real-time graph of magnetic strength over time (updates every second).
"""

import json
import threading
import time
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import tkinter as tk
from tkinter import ttk
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


class MagnetometerReceiver(Node):
    def __init__(self):
        super().__init__('magnetometer_receiver')

        # Current data
        self.current_data = {
            'x': 0.0, 'y': 0.0, 'z': 0.0,
            'magnitude': 0.0, 'unit': 'uT',
        }
        self.data_lock = threading.Lock()
        self.data_received = False

        # History for plotting (last 60 seconds)
        self.history_size = 60
        self.magnitude_history = deque(maxlen=self.history_size)
        self.x_history = deque(maxlen=self.history_size)
        self.y_history = deque(maxlen=self.history_size)
        self.z_history = deque(maxlen=self.history_size)
        self.time_history = deque(maxlen=self.history_size)
        self.start_time = time.time()

        # Subscriber
        self.subscription = self.create_subscription(
            String, 'magnetometer_data', self.data_callback, 10
        )

        self.get_logger().info('Magnetometer receiver started')

    def data_callback(self, msg):
        """Handle incoming magnetometer data."""
        try:
            data = json.loads(msg.data)
            with self.data_lock:
                self.current_data = data
                self.data_received = True
                self.magnitude_history.append(data.get('magnitude', 0.0))
                self.x_history.append(data.get('x', 0.0))
                self.y_history.append(data.get('y', 0.0))
                self.z_history.append(data.get('z', 0.0))
                self.time_history.append(time.time() - self.start_time)
        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().warn(f'Invalid data received: {e}')


class MagnetometerGUI:
    """Tkinter GUI that displays magnetometer data and a real-time graph."""

    def __init__(self, ros_node: MagnetometerReceiver):
        self.node = ros_node

        # --- Main window ---
        self.root = tk.Tk()
        self.root.title('Magnetometer Monitor')
        self.root.geometry('900x650')
        self.root.configure(bg='#1e1e2e')
        self.root.protocol('WM_DELETE_WINDOW', self.on_close)

        style = ttk.Style()
        style.theme_use('clam')
        style.configure('Title.TLabel', font=('Arial', 20, 'bold'),
                        background='#1e1e2e', foreground='#cdd6f4')
        style.configure('Data.TLabel', font=('Arial', 13),
                        background='#313244', foreground='#cdd6f4')
        style.configure('Mag.TLabel', font=('Arial', 22, 'bold'),
                        background='#45475a', foreground='#a6e3a1')
        style.configure('Status.TLabel', font=('Arial', 10),
                        background='#1e1e2e', foreground='#6c7086')
        style.configure('Card.TFrame', background='#313244')

        # Title
        ttk.Label(self.root, text='Magnetometer Monitor',
                  style='Title.TLabel').pack(pady=(12, 6))

        # --- Current values card ---
        card = ttk.Frame(self.root, style='Card.TFrame')
        card.pack(fill=tk.X, padx=16, pady=4)

        row = ttk.Frame(card, style='Card.TFrame')
        row.pack(fill=tk.X, padx=12, pady=8)

        self.x_var = tk.StringVar(value='X: --')
        self.y_var = tk.StringVar(value='Y: --')
        self.z_var = tk.StringVar(value='Z: --')

        for var in (self.x_var, self.y_var, self.z_var):
            ttk.Label(row, textvariable=var, style='Data.TLabel').pack(
                side=tk.LEFT, padx=20)

        # Magnitude display
        mag_frame = ttk.Frame(card, style='Card.TFrame')
        mag_frame.pack(fill=tk.X, padx=12, pady=(0, 10))

        ttk.Label(mag_frame, text='Magnetic Strength:',
                  style='Data.TLabel').pack()
        self.mag_var = tk.StringVar(value='-- uT')
        ttk.Label(mag_frame, textvariable=self.mag_var,
                  style='Mag.TLabel').pack(pady=4)

        # --- Intensity bar ---
        bar_frame = ttk.Frame(self.root, style='Card.TFrame')
        bar_frame.pack(fill=tk.X, padx=16, pady=4)

        ttk.Label(bar_frame, text='Intensity', style='Data.TLabel').pack(
            anchor=tk.W, padx=8, pady=(6, 0))
        self.intensity_canvas = tk.Canvas(bar_frame, height=24, bg='#45475a',
                                          highlightthickness=0)
        self.intensity_canvas.pack(fill=tk.X, padx=8, pady=(2, 8))

        # --- Graph ---
        graph_frame = ttk.Frame(self.root, style='Card.TFrame')
        graph_frame.pack(fill=tk.BOTH, expand=True, padx=16, pady=4)

        self.fig = Figure(figsize=(8, 3.5), dpi=100, facecolor='#313244')
        self.ax = self.fig.add_subplot(111)
        self._style_axis()

        self.canvas = FigureCanvasTkAgg(self.fig, graph_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=4, pady=4)

        # --- Status bar ---
        self.status_var = tk.StringVar(value='Waiting for data...')
        ttk.Label(self.root, textvariable=self.status_var,
                  style='Status.TLabel').pack(pady=(0, 6))

        # Start the periodic GUI update (every 1 000 ms)
        self.root.after(1000, self._update_gui)

    # ------------------------------------------------------------------ #
    def _style_axis(self):
        self.ax.set_facecolor('#45475a')
        self.ax.set_title('Magnetic Strength Over Time', color='#cdd6f4',
                          fontsize=12)
        self.ax.set_xlabel('Time (s)', color='#a6adc8', fontsize=10)
        self.ax.set_ylabel('Magnitude (uT)', color='#a6adc8', fontsize=10)
        self.ax.tick_params(colors='#a6adc8')
        for spine in self.ax.spines.values():
            spine.set_color('#585b70')
        self.ax.grid(True, alpha=0.25, color='#6c7086')

    # ------------------------------------------------------------------ #
    def _update_gui(self):
        """Refresh labels, intensity bar, and graph once per second."""
        with self.node.data_lock:
            data = dict(self.node.current_data)
            received = self.node.data_received
            mag_hist = list(self.node.magnitude_history)
            t_hist = list(self.node.time_history)

        if received:
            self.x_var.set(f"X: {data['x']:.1f}")
            self.y_var.set(f"Y: {data['y']:.1f}")
            self.z_var.set(f"Z: {data['z']:.1f}")
            unit = data.get('unit', 'uT')
            self.mag_var.set(f"{data['magnitude']:.2f} {unit}")
            self.status_var.set(
                f"Receiving data  |  Points: {len(mag_hist)}")

            # Intensity bar
            self._draw_intensity_bar(data['magnitude'])

            # Graph
            if len(mag_hist) > 1:
                self.ax.clear()
                self._style_axis()
                self.ax.plot(t_hist, mag_hist, color='#89b4fa', linewidth=2)
                self.ax.fill_between(t_hist, mag_hist, alpha=0.15,
                                     color='#89b4fa')
                self.fig.tight_layout()
                self.canvas.draw_idle()

        # Schedule next update
        self.root.after(1000, self._update_gui)

    def _draw_intensity_bar(self, magnitude):
        self.intensity_canvas.delete('all')
        w = self.intensity_canvas.winfo_width()
        h = self.intensity_canvas.winfo_height()
        if w < 2:
            return

        normalised = min(magnitude / 100.0, 1.0)
        bar_w = int(w * normalised)

        if magnitude < 25:
            color = '#a6e3a1'
        elif magnitude < 55:
            color = '#f9e2af'
        else:
            color = '#f38ba8'

        self.intensity_canvas.create_rectangle(0, 0, bar_w, h,
                                                fill=color, outline='')
        self.intensity_canvas.create_text(
            w // 2, h // 2, text=f'{magnitude:.1f} uT',
            fill='#1e1e2e', font=('Arial', 10, 'bold'))

    # ------------------------------------------------------------------ #
    def on_close(self):
        self.root.quit()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = MagnetometerReceiver()

    # Spin ROS in a background thread so the Tkinter mainloop is unblocked
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    gui = MagnetometerGUI(node)
    try:
        gui.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
