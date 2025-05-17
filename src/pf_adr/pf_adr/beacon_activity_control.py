import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int32
from rclpy.qos import QoSProfile
import time


class BeaconActivityMonitor(Node):
    def __init__(self):
        super().__init__('beacon_activity_monitor')

        # Parámetros configurables
        self.declare_parameter('num_beacons', 5)
        self.num_beacons = self.get_parameter('num_beacons').value

        self.declare_parameter('timeout_sec', 1.0)
        self.timeout_sec = self.get_parameter('timeout_sec').value

        self.last_seen = [0.0] * self.num_beacons
        self.subscribers = []

        for i in range(self.num_beacons):
            topic_name = f'/beacon_{i}/distance_to_target'
            sub = self.create_subscription(
                Float64,
                topic_name,
                lambda msg, i=i: self.distance_callback(msg, i),
                10
            )
            self.subscribers.append(sub)

        self.publisher = self.create_publisher(Int32, '/active_beacons_count', 10)
        self.timer = self.create_timer(0.5, self.check_active_beacons)

        self.get_logger().info(f'Monitor iniciado para {self.num_beacons} balizas.')

    def distance_callback(self, msg, beacon_id):
        if msg.data > 0.0 and msg.data < 10.0 and not msg.data == float('inf'):
            self.last_seen[beacon_id] = time.time()

    def check_active_beacons(self):
        now = time.time()
        active_count = sum((now - t) < self.timeout_sec for t in self.last_seen)
        msg = Int32()
        msg.data = active_count
        self.publisher.publish(msg)
        self.get_logger().info(f'{active_count} balizas activas')


def main(args=None):
    rclpy.init(args=args)
    node = BeaconActivityMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
