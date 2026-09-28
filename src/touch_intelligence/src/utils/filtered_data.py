import rclpy
import numpy as np

from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped


BASELINE_SAMPLES = 200

# -------------------------------
# ADDED CONSTANTS
# -------------------------------
RISE_THRESHOLD = 1.8

SEARCHING = 0
TRACKING = 1


class WrenchFilter(Node):

    def __init__(self):
        super().__init__("wrench_filter")

        self.sub = self.create_subscription(
            WrenchStamped,
            "/clean_ft_values",
            self.callback,
            10
        )

        self.pub = self.create_publisher(
            WrenchStamped,
            "/filtered_ft_values",
            10
        )

        self.history = {
            "fx": [],
            "fy": [],
            "fz": [],
            "tx": [],
            "ty": [],
            "tz": []
        }

        # -------------------------------
        # ADDED STATE VARIABLES
        # -------------------------------
        self.state = SEARCHING
        self.peak_fz = -np.inf
        self.peak_ty = 0.0

    def callback(self, msg):

        fx = msg.wrench.force.x
        fy = msg.wrench.force.y
        fz = msg.wrench.force.z

        tx = msg.wrench.torque.x
        ty = msg.wrench.torque.y
        tz = msg.wrench.torque.z

        self.history["fx"].append(fx)
        self.history["fy"].append(fy)
        self.history["fz"].append(fz)

        self.history["tx"].append(tx)
        self.history["ty"].append(ty)
        self.history["tz"].append(tz)

        if len(self.history["fx"]) < BASELINE_SAMPLES:
            return

        baseline_fx = np.mean(self.history["fx"])
        baseline_fy = np.mean(self.history["fy"])
        baseline_fz = np.mean(self.history["fz"])

        baseline_tx = np.mean(self.history["tx"])
        baseline_ty = np.mean(self.history["ty"])
        baseline_tz = np.mean(self.history["tz"])

        # -------------------------------
        # ADDED FILTERED VALUES (explicit variables)
        # -------------------------------
        fz_f = fz - baseline_fz
        ty_f = ty - baseline_ty

        # -------------------------------
        # ADDED PEAK DETECTION LOGIC
        # -------------------------------
        if self.state == SEARCHING:

            if fz_f > RISE_THRESHOLD:
                self.state = TRACKING
                self.peak_fz = fz_f
                self.peak_ty = ty_f

        elif self.state == TRACKING:

            # track peak Fz and corresponding Ty
            if fz_f > self.peak_fz:
                self.peak_fz = fz_f
                self.peak_ty = ty_f

            # impulse finished
            if fz_f < 0.5:

                delta_fz = self.peak_fz
                delta_ty = self.peak_ty

                ratio = 1.5 * delta_ty / delta_fz if abs(delta_fz) > 1e-6 else 0

                print(
                    f"ΔFz_peak: {delta_fz:.3f} | "
                    f"ΔTy_at_Fz_peak: {delta_ty:.3f} | "
                    f"1.5ΔTy/ΔFz: {ratio:.3f}"
                )

                self.state = SEARCHING

        # -------------------------------
        # ORIGINAL FILTERING + PUBLISH (UNCHANGED)
        # -------------------------------
        filtered_msg = WrenchStamped()
        filtered_msg.header = msg.header

        filtered_msg.wrench.force.x = fx - baseline_fx
        filtered_msg.wrench.force.y = fy - baseline_fy
        filtered_msg.wrench.force.z = fz - baseline_fz

        filtered_msg.wrench.torque.x = tx - baseline_tx
        filtered_msg.wrench.torque.y = ty - baseline_ty
        filtered_msg.wrench.torque.z = tz - baseline_tz

        self.pub.publish(filtered_msg)

        if len(self.history["fx"]) > BASELINE_SAMPLES:
            for key in self.history:
                self.history[key].pop(0)


def main(args=None):

    rclpy.init(args=args)

    node = WrenchFilter()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()