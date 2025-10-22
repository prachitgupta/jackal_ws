import argparse
import math
import signal
from typing import List, Tuple, Optional

import matplotlib
matplotlib.use("TkAgg")  # or 'Qt5Agg'
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Quaternion

class TrajectoryPlotter(Node):
    def __init__(
        self,
        odom_topic: str,
        vicon_topic: str,
        max_points: int = 20000,
        update_hz: float = 10.0,
        vicon_best_effort: bool = False,
        enable_odom: bool = True,
        enable_vicon: bool = True,
    ) -> None:
        super().__init__('trajectory_plotter')

        self.max_points = max_points
        self.enable_odom = enable_odom
        self.enable_vicon = enable_vicon

        self.odom_xy: List[Tuple[float, float]] = []
        self.vicon_xy: List[Tuple[float, float]] = []

        # Alignment offsets
        self.odom_offset: Optional[Tuple[float, float]] = None
        self.odom_theta_offset: Optional[float] = None
        self.vicon_start_quat: Optional[Quaternion] = None

        # QoS profiles
        odom_qos = QoSProfile(
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
        )
        vicon_qos = QoSProfile(
            depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT if vicon_best_effort else ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
        )

        if enable_odom:
            self.create_subscription(Odometry, odom_topic, self._odom_cb, odom_qos)
        if enable_vicon:
            self.create_subscription(PoseStamped, vicon_topic, self._vicon_cb, vicon_qos)

        # Plot setup
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.fig.canvas.manager.set_window_title('Trajectory Plotter')

        self.odom_line, = self.ax.plot([], [], label='Odometry /filtered', linewidth=2)
        self.vicon_line, = self.ax.plot([], [], label='Vicon Pose', linewidth=2, linestyle='--')

        self.ax.set_xlabel('X [m]')
        self.ax.set_ylabel('Y [m]')
        self.ax.set_aspect('equal', adjustable='box')
        self.ax.grid(True, linewidth=0.5, alpha=0.6)
        self.ax.legend(loc='best')

        period = 1.0 / update_hz if update_hz > 0 else 0.1
        self.timer = self.create_timer(period, self._update_plot)

    # -------------------- Callbacks --------------------
    def _odom_cb(self, msg: Odometry) -> None:
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y

        # Compute position offset once Vicon start is known
        if self.vicon_xy and self.odom_offset is None:
            v_start_x, v_start_y = self.vicon_xy[0]
            self.odom_offset = (v_start_x - x, v_start_y - y)

            # Compute rotation offset using first quaternions
            if self.vicon_start_quat is not None:
                odom_yaw = self._quaternion_to_yaw(msg.pose.pose.orientation)
                vicon_yaw = self._quaternion_to_yaw(self.vicon_start_quat)
                self.odom_theta_offset = vicon_yaw - odom_yaw -math.pi/2

        # Apply position offset
        if self.odom_offset is not None:
            x += self.odom_offset[0]
            y += self.odom_offset[1]

        # Apply rotation offset
        if self.odom_theta_offset is not None:
            x, y = self._rotate_point(x, y, self.vicon_xy[0], self.odom_theta_offset)

        self._append_point(self.odom_xy, (x, y))

    def _vicon_cb(self, msg: PoseStamped) -> None:
        x, y = msg.pose.position.x, msg.pose.position.y
        self._append_point(self.vicon_xy, (x, y))

        # Store first Vicon quaternion for rotation alignment
        if self.vicon_start_quat is None:
            self.vicon_start_quat = msg.pose.orientation

    # -------------------- Helpers --------------------
    def _append_point(self, buf: List[Tuple[float, float]], pt: Tuple[float, float]) -> None:
        buf.append(pt)
        if len(buf) > self.max_points:
            del buf[0: len(buf) - self.max_points]

    def _quaternion_to_yaw(self, q: Quaternion) -> float:
        """Convert quaternion to yaw (heading) in radians."""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _rotate_point(self, x: float, y: float, origin: Tuple[float, float], angle: float) -> Tuple[float, float]:
        """Rotate a point (x, y) around origin by angle (radians)."""
        ox, oy = origin
        dx, dy = x - ox, y - oy
        xr = dx * math.cos(angle) - dy * math.sin(angle)
        yr = dx * math.sin(angle) + dy * math.cos(angle)
        return ox + xr, oy + yr

    def _update_plot(self) -> None:
        if self.enable_odom and self.odom_xy:
            xs, ys = zip(*self.odom_xy)
            self.odom_line.set_data(xs, ys)
        if self.enable_vicon and self.vicon_xy:
            xs, ys = zip(*self.vicon_xy)
            self.vicon_line.set_data(xs, ys)

        all_pts = []
        if self.enable_odom:
            all_pts += self.odom_xy
        if self.enable_vicon:
            all_pts += self.vicon_xy

        if all_pts:
            xs, ys = zip(*all_pts)
            xmin, xmax = min(xs), max(xs)
            ymin, ymax = min(ys), max(ys)
            dx = max(1e-2, 0.05 * max(1.0, xmax - xmin))
            dy = max(1e-2, 0.05 * max(1.0, ymax - ymin))
            self.ax.set_xlim(xmin - dx, xmax + dx)
            self.ax.set_ylim(ymin - dy, ymax + dy)

        self.fig.canvas.draw()
        plt.pause(0.001)

    def save_png(self, path: Optional[str]) -> None:
        if path:
            try:
                self.fig.savefig(path, dpi=200, bbox_inches='tight')
                self.get_logger().info(f"Saved trajectory plot to: {path}")
            except Exception as e:
                self.get_logger().error(f"Failed to save plot: {e}")


def parse_args():
    parser = argparse.ArgumentParser(description='Plot Odometry and Vicon trajectories in ROS 2.')
    parser.add_argument('--odom_topic', default='/odometry/filtered', help='Odometry topic')
    parser.add_argument('--vicon_topic', default='/vicon/Jackal/Jackal/pose', help='Vicon Pose topic')
    parser.add_argument('--max_points', type=int, default=20000, help='Max stored points per trajectory')
    parser.add_argument('--update_hz', type=float, default=10.0, help='Plot refresh rate in Hz')
    parser.add_argument('--vicon_best_effort', action='store_true', help='Use BEST_EFFORT QoS for Vicon topic')
    parser.add_argument('--enable_odom', action='store_true', help='Enable Odometry subscription')
    parser.add_argument('--disable_odom', action='store_true', help='Disable Odometry subscription')
    parser.add_argument('--enable_vicon', action='store_true', help='Enable Vicon subscription')
    parser.add_argument('--disable_vicon', action='store_true', help='Disable Vicon subscription')
    parser.add_argument('--save', default='', help='Path to save PNG on exit')
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()

    # Determine which topics to subscribe to
    enable_odom = args.enable_odom or not args.disable_odom
    enable_vicon = args.enable_vicon or not args.disable_vicon

    node = TrajectoryPlotter(
        odom_topic=args.odom_topic,
        vicon_topic=args.vicon_topic,
        max_points=args.max_points,
        update_hz=args.update_hz,
        vicon_best_effort=args.vicon_best_effort,
        enable_odom=enable_odom,
        enable_vicon=enable_vicon,
    )

    # Graceful shutdown
    def _shutdown_handler(signum, frame):
        node.get_logger().info('Shutting down...')
        node.save_png(args.save if args.save else None)
        rclpy.shutdown()
        plt.ioff()
        try:
            plt.close(node.fig)
        except Exception:
            pass

    signal.signal(signal.SIGINT, _shutdown_handler)
    signal.signal(signal.SIGTERM, _shutdown_handler)

    try:
        rclpy.spin(node)
    finally:
        node.save_png(args.save if args.save else None)
        if rclpy.ok():
            rclpy.shutdown()
        plt.ioff()
        try:
            plt.close(node.fig)
        except Exception:
            pass


if __name__ == '__main__':
    main()
