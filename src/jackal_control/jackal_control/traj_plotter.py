import argparse
import math
import signal
from typing import List, Tuple, Optional
import matplotlib
matplotlib.use("TkAgg")  # or 'Qt5Agg' depending on your environment
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class TrajectoryPlotter(Node):
    def __init__(
        self,
        odom_topic: str,
        vicon_topic: str,
        max_points: int = 20000,
        update_hz: float = 10.0,
        vicon_best_effort: bool = False,
    ) -> None:
        super().__init__('trajectory_plotter')
        self.max_points = max_points
        self.odom_xy: List[Tuple[float, float]] = []
        self.vicon_xy: List[Tuple[float, float]] = []
        # QoS profiles: odometry often reliable; Vicon can be best-effort in some setups.
        odom_qos = QoSProfile(
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
        )
        vicon_qos = QoSProfile(
            depth=50,
            reliability=(ReliabilityPolicy.BEST_EFFORT if vicon_best_effort else ReliabilityPolicy.RELIABLE),
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Odometry, odom_topic, self._odom_cb, odom_qos)
        self.create_subscription(PoseStamped, vicon_topic, self._vicon_cb, vicon_qos)
        # Matplotlib live plot setup
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.fig.canvas.manager.set_window_title('Trajectory Plotter: Odometry vs Vicon')
        self.odom_line, = self.ax.plot([], [], label='Odometry /filtered', linewidth=2)
        self.vicon_line, = self.ax.plot([], [], label='Vicon Pose', linewidth=2, linestyle='--')
        self.ax.set_xlabel('X [m]')
        self.ax.set_ylabel('Y [m]')
        self.ax.set_aspect('equal', adjustable='box')
        self.ax.grid(True, linewidth=0.5, alpha=0.6)
        self.ax.legend(loc='best')
        # Timer to refresh the plot
        period = 1.0 / update_hz if update_hz > 0 else 0.1
        self.timer = self.create_timer(period, self._update_plot)
    # -------------------- Callbacks --------------------
    def _odom_cb(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self._append_point(self.odom_xy, (x, y))
    def _vicon_cb(self, msg: PoseStamped) -> None:
        x = msg.pose.position.x
        y = msg.pose.position.y
        self._append_point(self.vicon_xy, (x, y))
    # -------------------- Helpers --------------------
    def _append_point(self, buf: List[Tuple[float, float]], pt: Tuple[float, float]) -> None:
        buf.append(pt)
        if len(buf) > self.max_points:
            del buf[0: len(buf) - self.max_points]
    def _update_plot(self) -> None:
        # Update line data
        if self.odom_xy:
            xs, ys = zip(*self.odom_xy)
            self.odom_line.set_data(xs, ys)
        if self.vicon_xy:
            xs, ys = zip(*self.vicon_xy)
            self.vicon_line.set_data(xs, ys)
        # Autoscale view to data with margins
        all_pts = self.odom_xy + self.vicon_xy
        if all_pts:
            xs, ys = zip(*all_pts)
            xmin, xmax = min(xs), max(xs)
            ymin, ymax = min(ys), max(ys)
            # Add a small margin
            dx = max(1e-2, 0.05 * max(1.0, (xmax - xmin)))
            dy = max(1e-2, 0.05 * max(1.0, (ymax - ymin)))
            self.ax.set_xlim(xmin - dx, xmax + dx)
            self.ax.set_ylim(ymin - dy, ymax + dy)
        # Redraw
        self.fig.canvas.draw()
        plt.pause(0.001)  # allow GUI event loop to breathe
    def save_png(self, path: Optional[str]) -> None:
        if path:
            try:
                self.fig.savefig(path, dpi=200, bbox_inches='tight')
                self.get_logger().info(f"Saved trajectory plot to: {path}")
            except Exception as e:
                self.get_logger().error(f"Failed to save plot: {e}")
def parse_args():
    parser = argparse.ArgumentParser(description='Plot Odometry and Vicon trajectories in ROS 2.')
    parser.add_argument('--odom_topic', default='/odometry/filtered', help='Odometry topic (nav_msgs/Odometry)')
    parser.add_argument('--vicon_topic', default='/vicon/Jackal/Jackal/pose', help='Vicon Pose topic (geometry_msgs/PoseStamped)')
    parser.add_argument('--max_points', type=int, default=20000, help='Max stored points per trajectory')
    parser.add_argument('--update_hz', type=float, default=10.0, help='Plot refresh rate in Hz')
    parser.add_argument('--vicon_best_effort', action='store_true', help='Use BEST_EFFORT QoS for Vicon topic')
    parser.add_argument('--save', default='', help='Path to save PNG on exit')
    return parser.parse_args()
def main():
    args = parse_args()
    rclpy.init()
    node = TrajectoryPlotter(
        odom_topic=args.odom_topic,
        vicon_topic=args.vicon_topic,
        max_points=args.max_points,
        update_hz=args.update_hz,
        vicon_best_effort=args.vicon_best_effort,
    )
    # Graceful shutdown: save image if requested
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
        # In case shutdown was triggered elsewhere
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
