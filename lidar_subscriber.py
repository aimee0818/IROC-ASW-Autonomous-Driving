# lidar_subscriber.py
import threading

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from sensor_msgs.msg import LaserScan
    ROS2_AVAILABLE = True
except Exception:
    ROS2_AVAILABLE = False

class _LidarWatcher(Node):
    def __init__(self, topic='/scan'):
        super().__init__('lidar_watcher')
        self.latest_scan = None
        self.create_subscription(LaserScan, topic, self._cb, 10)
    def _cb(self, msg):
        self.latest_scan = msg

class LidarSubscriber:
    """ROS2 /scan 구독만 담당. latest_scan으로 접근."""
    def __init__(self, topic='/scan'):
        self.use_ros2 = ROS2_AVAILABLE
        self._node = None
        self._exec = None
        self._th = None
        if self.use_ros2:
            rclpy.init(args=None)
            self._node = _LidarWatcher(topic)
            self._exec = SingleThreadedExecutor()
            self._exec.add_node(self._node)
            self._th = threading.Thread(target=self._exec.spin, daemon=True)
            self._th.start()
            print(f"🔎 LiDAR subscriber started on {topic}")
        else:
            print("ℹ️ LiDAR subscriber disabled (ROS2 not available)")

    @property
    def latest_scan(self):
        if not self.use_ros2 or self._node is None:
            return None
        return self._node.latest_scan

    def shutdown(self):
        if self.use_ros2 and self._exec and self._node:
            try:
                self._exec.shutdown()
                self._node.destroy_node()
                rclpy.shutdown()
            except Exception:
                pass
