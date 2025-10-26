# ugv_bringup.py
# Robust JSON receive (brace-framed) + startup confirmation + RAW ESP32 PRINT

import serial
import json
import queue
import threading
import rclpy
from rclpy.node import Node
import logging
import time
from std_msgs.msg import Header, Float32MultiArray, Float32
from sensor_msgs.msg import Imu, MagneticField
import math
import os

serial_port = '/dev/ttyAMA0'

# ====================== RAW PRINT SWITCH (comment/flip to disable) ======================
DEBUG_PRINT_RAW = True  # Set to False or comment out the print block below to stop raw output
# ========================================================================================


class JsonFramer:
    """
    Streams bytes from serial and returns complete JSON objects framed by balanced { }.
    Tolerates garbage before/after, CRLF, concatenated frames, and split frames.
    """
    def __init__(self, ser):
        self.ser = ser
        self.buf = bytearray()

    def _fill(self):
        n = self.ser.in_waiting or 1
        chunk = self.ser.read(min(512, n))
        if chunk:
            self.buf.extend(chunk)

    def next_object(self, timeout_s=0.2):
        import time as _t
        end_time = _t.time() + timeout_s
        while _t.time() < end_time:
            start = self.buf.find(b'{')
            if start == -1:
                self._fill()
                continue
            if start > 0:
                del self.buf[:start]
            depth = 0
            for i, b in enumerate(self.buf):
                if b == ord('{'):
                    depth += 1
                elif b == ord('}'):
                    depth -= 1
                    if depth == 0:
                        obj = bytes(self.buf[:i+1])
                        del self.buf[:i+1]
                        return obj
            self._fill()
        return None


class BaseController:
    def __init__(self, uart_dev_set, baud_set):
        self.logger = logging.getLogger('BaseController')
        self.ser = serial.Serial(uart_dev_set, baud_set, timeout=0.02)
        time.sleep(0.1)
        self.ser.reset_input_buffer()
        self.framer = JsonFramer(self.ser)

        self.command_queue = queue.Queue()
        self.command_thread = threading.Thread(target=self.process_commands, daemon=True)
        self.command_thread.start()

        self.base_data = {"T": 1001, "L": 0, "R": 0, "ax": 0, "ay": 0, "az": 0,
                          "gx": 0, "gy": 0, "gz": 0, "mx": 0, "my": 0, "mz": 0,
                          "odl": 0, "odr": 0, "v": 0}

        print(f"[UGV_BRINGUP] UART initialized on {uart_dev_set} @ {baud_set} baud")

    def feedback_data(self):
        try:
            raw = self.framer.next_object(timeout_s=0.2)
            if raw is None:
                return None

            # ====================== RAW ESP32 OUTPUT (comment out to disable) ======================
            if DEBUG_PRINT_RAW:
                try:
                    print(f"[UGV_BRINGUP][RAW] {raw.decode('utf-8', 'replace').strip()}")
                except Exception as _e:
                    print(f"[UGV_BRINGUP][RAW] <{len(raw)} bytes, decode error>")
            # =======================================================================================

            obj = json.loads(raw.decode('utf-8', errors='ignore'))
            self.base_data = obj
            return obj
        except json.JSONDecodeError as e:
            print(f"[UGV_BRINGUP] JSON decode error: {e}")
            return None
        except Exception as e:
            print(f"[UGV_BRINGUP] Unexpected serial error: {e}")
            return None

    def send_command(self, data):
        self.command_queue.put(data)

    def process_commands(self):
        while True:
            data = self.command_queue.get()
            try:
                self.ser.write((json.dumps(data) + '\n').encode("utf-8"))
            except Exception as e:
                print(f"[UGV_BRINGUP] UART write failed: {e}")


class ugv_bringup(Node):
    def __init__(self):
        super().__init__('ugv_bringup')
        self.imu_data_raw_publisher_ = self.create_publisher(Imu, "imu/data_raw", 100)
        self.imu_mag_publisher_ = self.create_publisher(MagneticField, "imu/mag", 100)
        self.odom_publisher_ = self.create_publisher(Float32MultiArray, "odom/odom_raw", 100)
        self.voltage_publisher_ = self.create_publisher(Float32, "voltage", 50)
        self.base_controller = BaseController(serial_port, 115200)
        self.feedback_timer = self.create_timer(0.001, self.feedback_loop)
        print("[UGV_BRINGUP] Node initialized and running.")

    def feedback_loop(self):
        data = self.base_controller.feedback_data()
        if not data:
            return
        if data.get("T") == 1001:
            self.publish_imu_data_raw()
            self.publish_imu_mag()
            self.publish_odom_raw()
            self.publish_voltage()

    def publish_imu_data_raw(self):
        msg = Imu()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_imu_link"
        d = self.base_controller.base_data
        msg.linear_acceleration.x = 9.8 * float(d["ax"]) / 8192
        msg.linear_acceleration.y = 9.8 * float(d["ay"]) / 8192
        msg.linear_acceleration.z = 9.8 * float(d["az"]) / 8192
        msg.angular_velocity.x = math.pi * float(d["gx"]) / (16.4 * 180)
        msg.angular_velocity.y = math.pi * float(d["gy"]) / (16.4 * 180)
        msg.angular_velocity.z = math.pi * float(d["gz"]) / (16.4 * 180)
        self.imu_data_raw_publisher_.publish(msg)

    def publish_imu_mag(self):
        msg = MagneticField()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_imu_link"
        d = self.base_controller.base_data
        msg.magnetic_field.x = float(d["mx"]) * 0.15
        msg.magnetic_field.y = float(d["my"]) * 0.15
        msg.magnetic_field.z = float(d["mz"]) * 0.15
        self.imu_mag_publisher_.publish(msg)

    def publish_odom_raw(self):
        d = self.base_controller.base_data
        msg = Float32MultiArray(data=[d["odl"] / 100.0, d["odr"] / 100.0])
        self.odom_publisher_.publish(msg)

    def publish_voltage(self):
        d = self.base_controller.base_data
        msg = Float32()
        msg.data = float(d["v"]) / 100.0
        self.voltage_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    print("[UGV_BRINGUP] Starting node...")
    node = ugv_bringup()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[UGV_BRINGUP] Node interrupted, shutting down.")
    finally:
        rclpy.shutdown()
        print("[UGV_BRINGUP] Node stopped.")


if __name__ == '__main__':
    main()
