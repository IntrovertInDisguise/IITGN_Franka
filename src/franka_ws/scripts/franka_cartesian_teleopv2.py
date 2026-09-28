#!/usr/bin/env python3
from __future__ import annotations

import time
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

# NEW: pygame for smooth keydown/keyup-like behavior
import pygame

# KDL / URDF parsing
from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromUrdfModel
import PyKDL as kdl


# ============================================================
# Default key mapping (tune if needed)
# ============================================================

V_LIN_DEFAULT = 0.05   # m/s
V_ANG_DEFAULT = 0.30   # rad/s

# Internal twist ordering:
#   [vx, vy, vz, wx, wy, wz]
def make_key_mapping(v_lin: float, v_ang: float):
    return {
        # Translation
        "q": np.array([ v_lin, 0.0,   0.0,   0.0, 0.0, 0.0], dtype=float),  # +X
        "a": np.array([-v_lin, 0.0,   0.0,   0.0, 0.0, 0.0], dtype=float),  # -X
        "w": np.array([0.0,   v_lin,  0.0,   0.0, 0.0, 0.0], dtype=float),  # +Y
        "s": np.array([0.0,  -v_lin,  0.0,   0.0, 0.0, 0.0], dtype=float),  # -Y
        "e": np.array([0.0,   0.0,   v_lin,  0.0, 0.0, 0.0], dtype=float),  # +Z
        "d": np.array([0.0,   0.0,  -v_lin,  0.0, 0.0, 0.0], dtype=float),  # -Z

        # Rotation about base axes (roll/pitch/yaw)
        "r": np.array([0.0, 0.0, 0.0,  v_ang, 0.0, 0.0], dtype=float),      # +roll (X)
        "f": np.array([0.0, 0.0, 0.0, -v_ang, 0.0, 0.0], dtype=float),      # -roll
        "t": np.array([0.0, 0.0, 0.0, 0.0,  v_ang, 0.0], dtype=float),      # +pitch (Y)
        "g": np.array([0.0, 0.0, 0.0, 0.0, -v_ang, 0.0], dtype=float),      # -pitch
        "y": np.array([0.0, 0.0, 0.0, 0.0, 0.0,  v_ang], dtype=float),      # +yaw (Z)
        "h": np.array([0.0, 0.0, 0.0, 0.0, 0.0, -v_ang], dtype=float),      # -yaw
    }


class CartesianTeleopDLS(Node):
    def __init__(self):
        super().__init__("franka_cartesian_teleop_dls")

        # ------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------
        self.declare_parameter("joint_states_topic", "/NS_1/franka/joint_states")
        self.declare_parameter("cmd_topic", "/NS_1/joint_velocity_controller/commands")
        self.declare_parameter("robot_description_topic", "/NS_1/robot_description")

        self.declare_parameter("base_link", "fr3_link0")
        self.declare_parameter("tip_link", "fr3_link8")

        self.declare_parameter("rate_hz", 1000.0)

        self.declare_parameter("v_lin", V_LIN_DEFAULT)
        self.declare_parameter("v_ang", V_ANG_DEFAULT)

        self.declare_parameter("damping", 0.05)
        self.declare_parameter("joint_vel_limit", 0.7)

        self.declare_parameter("twist_frame", "base")
        self.declare_parameter("command_joint_names", [])

        # Pygame keyboard options
        self.declare_parameter("keyboard_hz", 200.0)
        self.declare_parameter("pygame_window_w", 360)
        self.declare_parameter("pygame_window_h", 160)
        self.declare_parameter("pygame_grab_input", True)  # helps keep keys from going to terminal

        # NEW: exit behavior
        self.declare_parameter("exit_key", "x")  # press-and-hold key to exit cleanly
        self.declare_parameter("exit_on_window_close", True)

        # ------------------------------------------------------------
        # Read params
        # ------------------------------------------------------------
        self.joint_states_topic = self.get_parameter("joint_states_topic").value
        self.cmd_topic = self.get_parameter("cmd_topic").value
        self.urdf_topic = self.get_parameter("robot_description_topic").value

        self.base_link = self.get_parameter("base_link").value
        self.tip_link = self.get_parameter("tip_link").value

        self.rate_hz = float(self.get_parameter("rate_hz").value)

        self.v_lin = float(self.get_parameter("v_lin").value)
        self.v_ang = float(self.get_parameter("v_ang").value)

        self.damping = float(self.get_parameter("damping").value)
        self.joint_vel_limit = float(self.get_parameter("joint_vel_limit").value)

        self.twist_frame = str(self.get_parameter("twist_frame").value).strip().lower()
        if self.twist_frame not in ("base", "ee"):
            self.twist_frame = "base"

        self.command_joint_names = list(self.get_parameter("command_joint_names").value)

        self.keyboard_hz = float(self.get_parameter("keyboard_hz").value)
        self.pygame_w = int(self.get_parameter("pygame_window_w").value)
        self.pygame_h = int(self.get_parameter("pygame_window_h").value)
        self.pygame_grab_input = bool(self.get_parameter("pygame_grab_input").value)

        self.exit_key_char = str(self.get_parameter("exit_key").value).strip().lower()[:1]
        if not self.exit_key_char.isprintable() or self.exit_key_char == "":
            self.exit_key_char = "x"
        self.exit_on_window_close = bool(self.get_parameter("exit_on_window_close").value)

        # Key mapping
        self.key_mapping = make_key_mapping(self.v_lin, self.v_ang)

        # Pygame key -> our char mapping (robust, no unicode dependency)
        self._pg_key_to_char = {
            pygame.K_q: "q",
            pygame.K_a: "a",
            pygame.K_w: "w",
            pygame.K_s: "s",
            pygame.K_e: "e",
            pygame.K_d: "d",
            pygame.K_r: "r",
            pygame.K_f: "f",
            pygame.K_t: "t",
            pygame.K_g: "g",
            pygame.K_y: "y",
            pygame.K_h: "h",
        }

        # NEW: exit key mapping (supports a..z, 0..9; falls back to x)
        self._exit_pg_key = self._pygame_key_from_char(self.exit_key_char)
        if self._exit_pg_key is None:
            self.exit_key_char = "x"
            self._exit_pg_key = pygame.K_x

        # ------------------------------------------------------------
        # State
        # ------------------------------------------------------------
        self._lock = threading.Lock()
        self.pressed_keys: set[str] = set()
        self.active_twist = np.zeros(6, dtype=float)

        self.current_js: JointState | None = None
        self._js_name_to_idx = None

        # KDL
        self.kdl_ready = False
        self.chain = None
        self.n_joints = 0
        self.chain_joint_names: list[str] = []
        self.jac_solver = None
        self.fk_solver = None

        # Logging throttle
        self._last_status_time = 0.0
        self._control_active_logged = False

        # Exit request (thread-safe)
        self._exit_requested = False

        # ------------------------------------------------------------
        # ROS interfaces
        # ------------------------------------------------------------
        self.cmd_pub = self.create_publisher(Float64MultiArray, self.cmd_topic, 10)
        self.create_subscription(JointState, self.joint_states_topic, self._joint_state_cb, 50)

        urdf_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._urdf_sub = self.create_subscription(String, self.urdf_topic, self._urdf_cb, urdf_qos)

        self.timer = self.create_timer(1.0 / self.rate_hz, self._control_loop)

        # ------------------------------------------------------------
        # Pygame keyboard thread
        # ------------------------------------------------------------
        self._kb_running = True
        self._kb_thread = threading.Thread(target=self._pygame_keyboard_loop, daemon=True)
        self._kb_thread.start()

        self.get_logger().info(
            "Cartesian DLS teleop started.\n"
            f"  Joint states: {self.joint_states_topic}\n"
            f"  Velocity cmds: {self.cmd_topic}\n"
            f"  URDF topic: {self.urdf_topic}\n"
            f"  base_link: {self.base_link}, tip_link: {self.tip_link}\n"
            f"  twist_frame: {self.twist_frame} (base or ee)\n"
            "Controls:\n"
            "  q/a : +X / -X\n"
            "  w/s : +Y / -Y\n"
            "  e/d : +Z / -Z\n"
            "  r/f : +roll / -roll\n"
            "  t/g : +pitch / -pitch\n"
            "  y/h : +yaw / -yaw\n"
            "  ESC : stop motion (clears keys)\n"
            f"  {self.exit_key_char.upper()}   : EXIT (clean shutdown)\n"
            "\nIMPORTANT: click the pygame window once so it has focus.\n"
        )

    # ---------------- Utilities ----------------

    @staticmethod
    def _pygame_key_from_char(ch: str) -> int | None:
        """
        Map a single character like 'x' to pygame.K_x.
        Supports a-z, 0-9. Returns None if unsupported.
        """
        if not ch:
            return None
        c = ch.lower()[0]
        if "a" <= c <= "z":
            return getattr(pygame, f"K_{c}", None)
        if "0" <= c <= "9":
            return getattr(pygame, f"K_{c}", None)
        return None

    def request_exit(self, reason: str = ""):
        # idempotent
        if self._exit_requested:
            return
        self._exit_requested = True
        if reason:
            self.get_logger().warn(f"Exit requested: {reason}")
        else:
            self.get_logger().warn("Exit requested.")
        # Stop motion immediately
        self._clear_keys()
        try:
            self._publish_joint_vel(np.zeros(len(self.command_joint_names), dtype=float))
        except Exception:
            pass
        # Stop keyboard loop
        self._kb_running = False

    # ---------------- URDF -> KDL ----------------

    def _urdf_cb(self, msg: String):
        if self.kdl_ready:
            return

        urdf_xml = msg.data.strip()
        if not urdf_xml or "<robot" not in urdf_xml:
            self.get_logger().error("robot_description received but doesn't look like URDF XML.")
            return

        self.get_logger().info(f"Received URDF on {self.urdf_topic}, building KDL chain...")

        try:
            robot = URDF.from_xml_string(urdf_xml)
            ok, tree = treeFromUrdfModel(robot)
            if not ok:
                raise RuntimeError("Failed to parse KDL tree from URDF.")

            chain = tree.getChain(self.base_link, self.tip_link)
            n = int(chain.getNrOfJoints())
            if n == 0:
                raise RuntimeError(
                    f"KDL chain has 0 joints for {self.base_link} -> {self.tip_link} (wrong link names?)."
                )

            # Extract ONLY movable joints, stop once we have exactly n
            names: list[str] = []
            for i in range(chain.getNrOfSegments()):
                seg = chain.getSegment(i)
                j = seg.getJoint()
                jt = j.getTypeName()
                if jt not in ("None", "Fixed"):
                    names.append(j.getName())
                    if len(names) == n:
                        break

            if len(names) != n:
                raise RuntimeError(
                    f"Joint-name extraction mismatch: chain.getNrOfJoints()={n} "
                    f"but extracted {len(names)} names: {names}"
                )

            self.chain = chain
            self.n_joints = n
            self.chain_joint_names = names
            self.jac_solver = kdl.ChainJntToJacSolver(self.chain)
            self.fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)

            if len(self.command_joint_names) == 0:
                self.command_joint_names = list(self.chain_joint_names)
            else:
                missing = [jn for jn in self.command_joint_names if jn not in set(self.chain_joint_names)]
                if missing:
                    self.get_logger().warn(
                        f"command_joint_names contains names not in KDL chain: {missing}. "
                        "Those commands will be output as 0.0."
                    )

            self.kdl_ready = True
            self.get_logger().info(
                f"KDL ready: n_joints={self.n_joints}\n"
                f"  chain_joint_names={self.chain_joint_names}\n"
                f"  command_joint_names={self.command_joint_names}"
            )

            self.destroy_subscription(self._urdf_sub)

        except Exception as e:
            self.get_logger().error(f"Failed to build KDL chain from URDF: {e}")

    # ---------------- Joint states ----------------

    def _joint_state_cb(self, msg: JointState):
        self.current_js = msg
        self._js_name_to_idx = {n: i for i, n in enumerate(msg.name)}

    # ---------------- Keyboard (pygame robust) ----------------

    def _recompute_twist(self):
        tw = np.zeros(6, dtype=float)
        for k in self.pressed_keys:
            vec = self.key_mapping.get(k, None)
            if vec is not None:
                tw += vec
        self.active_twist = tw

    def _clear_keys(self):
        with self._lock:
            self.pressed_keys.clear()
            self._recompute_twist()

    def _pygame_keyboard_loop(self):
        """
        Robust, smooth "hold" teleop:
          - Uses pygame.key.get_pressed() (no unicode dependence)
          - Clears on focus loss
          - Optional input grab to prevent terminal getting the keys
          - NEW: Exit key (default 'x') triggers clean shutdown
        """
        try:
            pygame.init()
            screen = pygame.display.set_mode((self.pygame_w, self.pygame_h))
            pygame.display.set_caption("Teleop Keys (CLICK HERE)")
            clock = pygame.time.Clock()

            # Grab input so keystrokes don't go to your terminal
            try:
                pygame.event.set_grab(self.pygame_grab_input)
            except Exception:
                pass

            font = pygame.font.Font(None, 22)

            info = [
                "CLICK THIS WINDOW (focus required).",
                "Hold keys -> continuous motion.",
                "ESC clears keys / stops motion.",
                f"{self.exit_key_char.upper()} exits (clean shutdown).",
                "Close window exits (if enabled).",
            ]

            self.get_logger().info(
                "Pygame teleop running. If you see 'wwww' in terminal, the window is NOT focused."
            )

            while rclpy.ok() and self._kb_running and (not self._exit_requested):
                # Pump events (required for key states to update)
                pygame.event.pump()

                # Handle quit
                for event in pygame.event.get([pygame.QUIT]):
                    if event.type == pygame.QUIT:
                        if self.exit_on_window_close:
                            self.request_exit("Pygame window closed.")
                        else:
                            self.get_logger().warn("Pygame window closed -> stopping keyboard thread.")
                            self._kb_running = False
                            self._clear_keys()
                        break

                if (not self._kb_running) or self._exit_requested:
                    break

                # If window not focused, clear motion
                if not pygame.key.get_focused():
                    self._clear_keys()
                else:
                    keys = pygame.key.get_pressed()

                    # NEW: Exit key
                    if self._exit_pg_key is not None and keys[self._exit_pg_key]:
                        self.request_exit(f"Exit key '{self.exit_key_char}' pressed.")
                    # ESC: clear
                    elif keys[pygame.K_ESCAPE]:
                        self._clear_keys()
                    else:
                        held = set()
                        for pgk, ch in self._pg_key_to_char.items():
                            if keys[pgk]:
                                held.add(ch)

                        with self._lock:
                            self.pressed_keys = held
                            self._recompute_twist()

                # Tiny UI (also keeps window responsive)
                screen.fill((25, 25, 25))
                y = 10
                for line in info:
                    screen.blit(font.render(line, True, (230, 230, 230)), (10, y))
                    y += 24

                with self._lock:
                    keys_str = "".join(sorted(self.pressed_keys)) if self.pressed_keys else "(none)"
                screen.blit(font.render(f"Pressed: {keys_str}", True, (180, 220, 180)), (10, y + 4))

                pygame.display.flip()
                clock.tick(max(60.0, self.keyboard_hz))

        except Exception as e:
            self.get_logger().error(f"Pygame keyboard loop crashed: {e}")
            self._clear_keys()
        finally:
            try:
                pygame.quit()
            except Exception:
                pass

    # ---------------- Helpers ----------------

    def _publish_joint_vel(self, qdot_cmd: np.ndarray):
        msg = Float64MultiArray()
        msg.data = qdot_cmd.tolist()
        self.cmd_pub.publish(msg)

    def _get_q_chain(self) -> np.ndarray | None:
        if (self.current_js is None) or (self._js_name_to_idx is None):
            return None

        q_chain = np.zeros(self.n_joints, dtype=float)
        missing = False

        for i in range(self.n_joints):
            jn = self.chain_joint_names[i]
            idx = self._js_name_to_idx.get(jn, None)
            if idx is None:
                missing = True
                q_chain[i] = 0.0
            else:
                q_chain[i] = float(self.current_js.position[idx])

        if missing:
            now = time.time()
            if now - self._last_status_time > 2.0:
                self.get_logger().warn(
                    "Some chain_joint_names were missing in JointState. "
                    "This will break Cartesian control. Check naming consistency!"
                )
                self._last_status_time = now

        return q_chain

    def _chain_to_cmd_order(self, qdot_chain: np.ndarray) -> np.ndarray:
        name_to_chain_idx = {n: i for i, n in enumerate(self.chain_joint_names)}
        qdot_cmd = np.zeros(len(self.command_joint_names), dtype=float)
        for k, jn in enumerate(self.command_joint_names):
            ci = name_to_chain_idx.get(jn, None)
            qdot_cmd[k] = float(qdot_chain[ci]) if ci is not None else 0.0
        return qdot_cmd

    def _twist_ee_to_base(self, q_chain: np.ndarray, twist_ee: np.ndarray) -> np.ndarray:
        q_kdl = kdl.JntArray(self.n_joints)
        for i in range(self.n_joints):
            q_kdl[i] = float(q_chain[i])

        frame = kdl.Frame()
        self.fk_solver.JntToCart(q_kdl, frame)

        Rb = np.array([
            [frame.M[0, 0], frame.M[0, 1], frame.M[0, 2]],
            [frame.M[1, 0], frame.M[1, 1], frame.M[1, 2]],
            [frame.M[2, 0], frame.M[2, 1], frame.M[2, 2]],
        ], dtype=float)

        v_lin_base = Rb @ twist_ee[:3]
        v_ang_base = Rb @ twist_ee[3:]
        return np.concatenate([v_lin_base, v_ang_base], axis=0)

    # ---------------- Control loop ----------------

    def _control_loop(self):
        # NEW: allow keyboard thread to trigger clean shutdown
        if self._exit_requested:
            self.get_logger().warn("Shutting down (exit requested).")
            try:
                self._publish_joint_vel(np.zeros(len(self.command_joint_names), dtype=float))
            except Exception:
                pass
            # Stop spinning this node
            try:
                rclpy.shutdown()
            except Exception:
                pass
            return

        if not self.kdl_ready:
            return
        if (self.current_js is None) or (self._js_name_to_idx is None):
            return

        q_chain = self._get_q_chain()
        if q_chain is None:
            return

        if not self._control_active_logged:
            self.get_logger().info("Control loop ACTIVE (KDL + joint_states ready).")
            self._control_active_logged = True

        with self._lock:
            twist = self.active_twist.copy()

        if np.allclose(twist, 0.0):
            self._publish_joint_vel(np.zeros(len(self.command_joint_names), dtype=float))
            return

        if self.twist_frame == "ee":
            twist = self._twist_ee_to_base(q_chain, twist)

        q_kdl = kdl.JntArray(self.n_joints)
        for i in range(self.n_joints):
            q_kdl[i] = float(q_chain[i])

        jac_kdl = kdl.Jacobian(self.n_joints)
        self.jac_solver.JntToJac(q_kdl, jac_kdl)

        J = np.zeros((6, self.n_joints), dtype=float)
        for i in range(6):
            for j in range(self.n_joints):
                J[i, j] = jac_kdl[i, j]

        v = twist.reshape(6, 1)
        lam = float(self.damping)
        A = (J @ J.T) + (lam * lam) * np.eye(6)

        try:
            qdot_chain = (J.T @ np.linalg.solve(A, v)).flatten()
        except np.linalg.LinAlgError:
            self.get_logger().error("DLS solve failed (singular).")
            self._publish_joint_vel(np.zeros(len(self.command_joint_names), dtype=float))
            return

        qdot_chain = np.clip(qdot_chain, -self.joint_vel_limit, self.joint_vel_limit)
        qdot_cmd = self._chain_to_cmd_order(qdot_chain)

        now = time.time()
        if now - self._last_status_time > 0.5:
            self._last_status_time = now
            self.get_logger().info(
                f"twist[{self.twist_frame}]={np.round(twist,3)} | ||qdot||={np.linalg.norm(qdot_chain):.3f}"
            )

        self._publish_joint_vel(qdot_cmd)

    # ---------------- Cleanup ----------------

    def destroy_node(self):
        try:
            self._publish_joint_vel(np.zeros(len(self.command_joint_names), dtype=float))
        except Exception:
            pass

        self._kb_running = False
        self._clear_keys()

        super().destroy_node()


def main():
    rclpy.init()
    node = CartesianTeleopDLS()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
