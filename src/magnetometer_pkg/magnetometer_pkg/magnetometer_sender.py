#!/usr/bin/env python3
"""
ROS2 Magnetometer Sender Node
Reads magnetometer data from serial port and publishes it once per second.
Supports labeled text lines, CSV lines, and raw binary input.
"""

import json
import math
import re
import struct
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial


class MagnetometerSender(Node):
    def __init__(self):
        super().__init__('magnetometer_sender')

        # Declare parameters
        self.declare_parameter('port', 'auto')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 1.0)

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value

        # Publisher
        self.publisher = self.create_publisher(String, 'magnetometer_data', 10)

        # Serial connection
        self.serial_connection = None
        self.connected_port = None

        # Regex pattern for parsing magnetometer data
        # Format: X: 77, Y: -165, Z: -546, Mag: 52.80 uT
        self.data_pattern = re.compile(
            r'X:\s*(-?\d+(?:\.\d+)?),?\s*Y:\s*(-?\d+(?:\.\d+)?),?\s*'
            r'Z:\s*(-?\d+(?:\.\d+)?),?\s*Mag:\s*(-?\d+(?:\.\d+)?)\s*(\w+)'
        )

        # Latest parsed data (updated from serial reads)
        self.latest_data = None

        # Connect to serial port
        self.connect_serial()

        # Timer to read serial data frequently (10 Hz)
        self.read_timer = self.create_timer(0.1, self.read_serial)

        # Timer to publish data once per second
        self.publish_timer = self.create_timer(1.0, self.publish_data)

        self.get_logger().info(
            f'Magnetometer sender started on port {self.port} at {self.baudrate} baud'
        )

    def connect_serial(self):
        """Establish serial connection."""
        port = self.resolve_port(self.port)
        if not port:
            self.get_logger().error(
                'No serial device found. Set the port parameter explicitly '
                'or plug in the magnetometer adapter.'
            )
            self.serial_connection = None
            return

        try:
            self.serial_connection = serial.Serial(
                port=port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
            )
            self.connected_port = port
            self.get_logger().info(f'Connected to {port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to {port}: {e}')
            self.serial_connection = None

    def resolve_port(self, port):
        """Resolve the serial port to use."""
        if port and port != 'auto':
            return port

        by_id_dir = Path('/dev/serial/by-id')
        if by_id_dir.exists():
            candidates = sorted(p for p in by_id_dir.iterdir() if p.exists())
            if candidates:
                return str(candidates[0])

        for pattern in ('/dev/ttyUSB*', '/dev/ttyACM*'):
            candidates = sorted(Path('/dev').glob(pattern.replace('/dev/', '')))
            if candidates:
                return str(candidates[0])

        return None

    def parse_text_line(self, line):
        """Parse a text line in format: X: 77, Y: -165, Z: -546, Mag: 52.80 uT"""
        match = self.data_pattern.match(line.strip())
        if match:
            x = float(match.group(1))
            y = float(match.group(2))
            z = float(match.group(3))
            magnitude = float(match.group(4))
            unit = match.group(5)
            return {'x': x, 'y': y, 'z': z, 'magnitude': magnitude, 'unit': unit}
        return None

    def parse_csv_line(self, line):
        """Parse a CSV line in format: x,y,z."""
        parts = [part.strip() for part in line.strip().split(',')]
        if len(parts) != 3:
            return None

        try:
            x = float(parts[0])
            y = float(parts[1])
            z = float(parts[2])
        except ValueError:
            return None

        magnitude = math.sqrt(x * x + y * y + z * z)
        return {'x': x, 'y': y, 'z': z, 'magnitude': magnitude, 'unit': 'uT'}

    def parse_binary_data(self, raw_data):
        """Try to parse binary data as X, Y, Z floats or int16s."""
        if len(raw_data) >= 12:
            try:
                x, y, z = struct.unpack('<fff', raw_data[:12])
                magnitude = math.sqrt(x * x + y * y + z * z)
                return {'x': x, 'y': y, 'z': z, 'magnitude': magnitude, 'unit': 'uT'}
            except struct.error:
                pass
        if len(raw_data) >= 6:
            try:
                x, y, z = struct.unpack('<hhh', raw_data[:6])
                magnitude = math.sqrt(x * x + y * y + z * z)
                return {
                    'x': float(x), 'y': float(y), 'z': float(z),
                    'magnitude': magnitude, 'unit': 'uT',
                }
            except struct.error:
                pass
        return None

    def read_serial(self):
        """Read data from serial port and store latest reading."""
        if self.serial_connection is None or not self.serial_connection.is_open:
            # Attempt reconnection
            self.connect_serial()
            return

        try:
            if self.serial_connection.in_waiting > 0:
                raw_line = self.serial_connection.readline()

                # Try decoding as text first
                try:
                    line = raw_line.decode('utf-8').strip()
                except UnicodeDecodeError:
                    try:
                        line = raw_line.decode('latin-1').strip()
                    except UnicodeDecodeError:
                        line = None

                if line:
                    data = self.parse_text_line(line)
                    if data:
                        self.latest_data = data
                        return

                    data = self.parse_csv_line(line)
                    if data:
                        self.latest_data = data
                        return

                # Fall back to binary parsing
                data = self.parse_binary_data(raw_line)
                if data:
                    self.latest_data = data

        except serial.SerialException as e:
            self.get_logger().warn(f'Serial read error: {e}')
            self.serial_connection = None

    def publish_data(self):
        """Publish the latest magnetometer data as JSON."""
        if self.latest_data is None:
            return

        msg = String()
        msg.data = json.dumps(self.latest_data)
        self.publisher.publish(msg)

        d = self.latest_data
        self.get_logger().info(
            f"Published: X={d['x']:.1f}, Y={d['y']:.1f}, Z={d['z']:.1f}, "
            f"Mag={d['magnitude']:.2f} {d['unit']}"
        )

    def destroy_node(self):
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MagnetometerSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
