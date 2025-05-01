import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
import numpy as np

class BeaconEstimator(Node):
    def __init__(self):
        super().__init__('beacon_particle_filter')

        self.declare_parameter('num_particles', 200)
        self.declare_parameter('init_spread', 10.0)

        self.num_particles = self.get_parameter('num_particles').value
        self.spread = self.get_parameter('init_spread').value

        self.particles = np.random.uniform(-self.spread, self.spread, (self.num_particles, 2))  # (x, y)
        self.weights = np.ones(self.num_particles) / self.num_particles

        self.drone_position = None
        self.measured_distance = None

        self.create_subscription(Odometry, '/simple_drone/odom', self.odom_callback, 10)
        self.create_subscription(Float64, '/distance_to_target', self.distance_callback, 10)

        self.pose_pub = self.create_publisher(PoseStamped, '/pf/beacon_estimate', 10)
        self.particles_pub = self.create_publisher(PoseArray, '/pf/particles', 10)

        self.timer = self.create_timer(0.1, self.update)

        self.get_logger().info('Beacon particle filter node initialized.')

    def odom_callback(self, msg):
        self.drone_position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ])

    def distance_callback(self, msg):
        self.measured_distance = msg.data

    def update(self):
        if self.drone_position is None or self.measured_distance is None:
            return

        # Calcular distancia desde cada partícula al dron
        dists = np.linalg.norm(self.particles - self.drone_position, axis=1)

        # Comparar con la distancia medida
        error = dists - self.measured_distance
        self.weights = np.exp(-0.5 * (error ** 2) / 0.5**2)
        self.weights += 1e-30  # evitar ceros
        self.weights /= np.sum(self.weights)

        # Re-muestreo
        indices = np.random.choice(range(self.num_particles), self.num_particles, p=self.weights)
        self.particles = self.particles[indices]
        self.weights.fill(1.0 / self.num_particles)

        # Estimar posición promedio
        estimate = np.mean(self.particles, axis=0)
        self.publish_estimate(estimate)
        self.publish_particles()

    def publish_estimate(self, estimate):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = estimate[0]
        pose.pose.position.y = estimate[1]
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose)

    def publish_particles(self):
        pa = PoseArray()
        pa.header.stamp = self.get_clock().now().to_msg()
        pa.header.frame_id = 'map'
        for p in self.particles:
            pose = Pose()
            pose.position.x = p[0]
            pose.position.y = p[1]
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            pa.poses.append(pose)
        self.particles_pub.publish(pa)

def main(args=None):
    rclpy.init(args=args)
    node = BeaconEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
