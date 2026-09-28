#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from pynput import keyboard

# ============================================================
# Config
# ============================================================

# Nominal joint velocity (rad/s)
V = 0.1

# 7-DOF mapping: q/a, w/s, e/d, r/f, t/g, y/h, u/j
KEY_MAPPING = {
    "q": [ V, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "a": [-V, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],

    "w": [0.0,  V, 0.0, 0.0, 0.0, 0.0, 0.0],
    "s": [0.0, -V, 0.0, 0.0, 0.0, 0.0, 0.0],

    "e": [0.0, 0.0,  V, 0.0, 0.0, 0.0, 0.0],
    "d": [0.0, 0.0, -V, 0.0, 0.0, 0.0, 0.0],

    "r": [0.0, 0.0, 0.0,  V, 0.0, 0.0, 0.0],
    "f": [0.0, 0.0, 0.0, -V, 0.0, 0.0, 0.0],

    "t": [0.0, 0.0, 0.0, 0.0,  V, 0.0, 0.0],
    "g": [0.0, 0.0, 0.0, 0.0, -V, 0.0, 0.0],

    "y": [0.0, 0.0, 0.0, 0.0, 0.0,  V, 0.0],
    "h": [0.0, 0.0, 0.0, 0.0, 0.0, -V, 0.0],

    "u": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  V],
    "j": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -V],
}

N_JOINTS = 7


def vec_add(a, b):
    return [x + y for x, y in zip(a, b)]


class KeyboardVelocityControl(Node):
    def __init__(self):
        super().__init__("franka_keyboard_velocity_control")

        # Topic that matches your controller:  /NS_1/joint_velocity_controller/commands
        # If you ever change namespace / controller name, just tweak this.
        self.topic = "/NS_1/joint_velocity_controller/commands"
        self.pub = self.create_publisher(Float64MultiArray, self.topic, 10)

        self.n_joints = N_JOINTS
        self.active_command = [0.0] * self.n_joints
        self.pressed_keys = set()

        # 50 Hz publishing
        self.timer = self.create_timer(1.0 / 50.0, self.publish_cmd)

        # Start keyboard listener in background thread
        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release,
        )
        self.listener.start()

        self.get_logger().info(
            f"Keyboard teleop started.\n"
            f"  Topic: {self.topic}\n"
            f"  Rate:  50 Hz\n"
            f"Controls:\n"
            f"  q/a, w/s, e/d, r/f, t/g, y/h, u/j for joints 1..7\n"
            f"  ESC to quit."
        )

    # ---------------- Keyboard handling ----------------

    def update_active_command(self):
        cmd = [0.0] * self.n_joints
        for k in self.pressed_keys:
            if k in KEY_MAPPING:
                cmd = vec_add(cmd, KEY_MAPPING[k])
        self.active_command = cmd

    def on_press(self, key):
        try:
            if key == keyboard.Key.esc:
                # Let on_release handle clean shutdown
                return

            if hasattr(key, "char") and key.char:
                ch = key.char.lower()
                if ch in KEY_MAPPING:
                    self.pressed_keys.add(ch)
                    self.update_active_command()
        except Exception:
            pass

    def on_release(self, key):
        try:
            if key == keyboard.Key.esc:
                # Stop ROS cleanly
                self.get_logger().info("ESC pressed, shutting down teleop.")
                rclpy.try_shutdown()
                return False  # stop listener

            if hasattr(key, "char") and key.char:
                ch = key.char.lower()
                if ch in self.pressed_keys:
                    self.pressed_keys.remove(ch)
                    self.update_active_command()
        except Exception:
            pass

    # ---------------- ROS timer callback ----------------

    def publish_cmd(self):
        msg = Float64MultiArray()
        msg.data = self.active_command
        self.pub.publish(msg)
        # Uncomment if you want spammy logs:
        # self.get_logger().info(f"cmd = {self.active_command}")


def main():
    rclpy.init()
    node = KeyboardVelocityControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.listener.stop()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
