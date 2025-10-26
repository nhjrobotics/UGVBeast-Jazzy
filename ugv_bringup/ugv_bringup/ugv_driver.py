# ugv_driver.py
# Added startup confirmation and runtime prints

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import json
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float32MultiArray
import subprocess
import time

serial_port = '/dev/ttyAMA0'
ser = serial.Serial(serial_port, 115200, timeout=1)
print(f"[UGV_DRIVER] UART initialized on {serial_port} @115200 baud")


class UgvDriver(Node):
    def __init__(self, name):
        super().__init__(name)
        self.cmd_vel_sub_ = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)
        self.joint_states_sub = self.create_subscription(JointState, 'ugv/joint_states', self.joint_states_callback, 10)
        self.led_ctrl_sub = self.create_subscription(Float32MultiArray, 'ugv/led_ctrl', self.led_ctrl_callback, 10)
        self.voltage_sub = self.create_subscription(Float32, 'voltage', self.voltage_callback, 10)
        print("[UGV_DRIVER] Node initialized and running.")

    def cmd_vel_callback(self, msg):
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z
        if linear_velocity == 0:
            if 0 < angular_velocity < 0.2:
                angular_velocity = 0.2
            elif -0.2 < angular_velocity < 0:
                angular_velocity = -0.2
        data = json.dumps({'T': '13', 'X': linear_velocity, 'Z': angular_velocity}) + "\n"
        ser.write(data.encode())
        print(f"[UGV_DRIVER] cmd_vel sent: X={linear_velocity:.2f}, Z={angular_velocity:.2f}")

    def joint_states_callback(self, msg):
        name = msg.name
        position = msg.position
        x_rad = position[name.index('pt_base_link_to_pt_link1')]
        y_rad = position[name.index('pt_link1_to_pt_link2')]
        x_degree = (180.0 * x_rad) / 3.1415926
        y_degree = (180.0 * y_rad) / 3.1415926
        joint_data = json.dumps({'T': 134, 'X': x_degree, 'Y': y_degree, 'SX': 600, 'SY': 600}) + "\n"
        ser.write(joint_data.encode())
        print(f"[UGV_DRIVER] Joint state sent: X={x_degree:.1f}, Y={y_degree:.1f}")

    def led_ctrl_callback(self, msg):
        IO4, IO5 = msg.data[0], msg.data[1]
        led_ctrl_data = json.dumps({'T': 132, 'IO4': IO4, 'IO5': IO5}) + "\n"
        ser.write(led_ctrl_data.encode())
        print(f"[UGV_DRIVER] LED control sent: IO4={IO4}, IO5={IO5}")

    def voltage_callback(self, msg):
        voltage_value = msg.data
        if 0.1 < voltage_value < 9:
            print(f"[UGV_DRIVER] Low voltage detected ({voltage_value:.2f}V) — playing warning.")
            subprocess.run(
                ['aplay', '-D', 'plughw:3,0',
                 '/home/ws/ugv_ws/src/ugv_main/ugv_bringup/ugv_bringup/low_battery.wav'],
                check=False)
            time.sleep(5)


def main(args=None):
    rclpy.init(args=args)
    print("[UGV_DRIVER] Starting node...")
    node = UgvDriver("ugv_driver")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[UGV_DRIVER] Node interrupted, shutting down.")
    finally:
        node.destroy_node()
        ser.close()
        rclpy.shutdown()
        print("[UGV_DRIVER] Node stopped.")


if __name__ == '__main__':
    main()
