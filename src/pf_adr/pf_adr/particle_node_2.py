import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
import numpy as np

class BeaconParticleFilter(Node):
    def __init__(self):
        super().__init__('particle_filter_node')

        # Parámetro beacon_id
        self.declare_parameter('beacon_id', 0)
        self.beacon_id = self.get_parameter('beacon_id').value

        # Parámetros configurables
        # Nuevos parámetros
        self.declare_parameter('total_num_particles', 5000)
        self.declare_parameter('total_beacons', 5)

        self.total_num_particles = self.get_parameter('total_num_particles').value
        self.total_beacons = self.get_parameter('total_beacons').value

        # Reparto uniforme
        self.num_particles = self.total_num_particles // self.total_beacons

        self.declare_parameter('sigma', 0.02)
        self.declare_parameter('noise_std', 0.02)
        self.declare_parameter('radius', 1.0)
        self.declare_parameter('init_x_range', [-1.0, 1.0])
        self.declare_parameter('init_y_range', [-1.0, 1.0])
        self.declare_parameter('init_z_range', [0.0, 1.0])

        self.num_particles = self.get_parameter('num_particles').value
        self.sigma = self.get_parameter('sigma').value
        self.noise_std = self.get_parameter('noise_std').value
        self.radius = self.get_parameter('radius').value
        self.init_x_range = self.get_parameter('init_x_range').value
        self.init_y_range = self.get_parameter('init_y_range').value
        self.init_z_range = self.get_parameter('init_z_range').value

        self.particles = self.initialize_particles2()
        self.weights = np.ones(self.num_particles) / self.num_particles

        self.current_drone_position = None
        self.current_distance = None

        # Topic distance_to_target específico de la baliza
        distance_topic = f'/beacon_{self.beacon_id}/distance_to_target'

        # Subscripciones
        self.create_subscription(Odometry, '/simple_drone/odom', self.odom_callback, 10)
        self.create_subscription(Float64, distance_topic, self.distance_callback, 10)

        # Publicadores con topics específicos
        self.pose_pub = self.create_publisher(PoseStamped, f'/pf/beacon_{self.beacon_id}/estimate', 10)
        self.particles_pub = self.create_publisher(PoseArray, f'/pf/beacon_{self.beacon_id}/particles', 10)

        self.timer = self.create_timer(0.05, self.update_filter)
        self.get_logger().info(f'Filtro de partículas para baliza {self.beacon_id} inicializado.')

    def initialize_particles2(self):
        phi = np.random.uniform(0, 2 * np.pi, self.num_particles)
        costheta = np.random.uniform(-1, 1, self.num_particles)
        u = np.random.uniform(0, 1, self.num_particles)
        theta = np.arccos(costheta)
        r = self.radius * u**(1/3)
        x = r * np.sin(theta) * np.cos(phi)
        y = r * np.sin(theta) * np.sin(phi)
        z = r * costheta
        return np.stack([x, y, z], axis=-1)

    def odom_callback(self, msg):
        self.current_drone_position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])

    def distance_callback(self, msg):
        self.current_distance = msg.data

    def update_filter(self):
        if self.current_drone_position is None or self.current_distance is None:
            return

        diffs = self.particles - self.current_drone_position
        dists = np.linalg.norm(diffs, axis=1)

        self.weights = np.exp(-0.5 * ((dists - self.current_distance) ** 2) / self.sigma**2)
        self.weights += 1e-300
        self.weights /= np.sum(self.weights)

        if self.effective_sample_size() < self.num_particles / 2:
            self.resample_particles()

        self.publish_estimate()
        self.publish_particles()

    def effective_sample_size(self):
        return 1.0 / np.sum(np.square(self.weights))

    def resample_particles(self):
        cumulative_sum = np.cumsum(self.weights)
        cumulative_sum[-1] = 1.0
        indices = np.searchsorted(cumulative_sum, np.random.rand(self.num_particles))
        self.particles = self.particles[indices]
        noise = np.random.normal(0, self.noise_std, self.particles.shape)
        self.particles += noise
        self.weights.fill(1.0 / self.num_particles)

    def publish_estimate(self):
        mean = np.average(self.particles, axis=0, weights=self.weights)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = mean[0]
        msg.pose.position.y = mean[1]
        msg.pose.position.z = mean[2]
        msg.pose.orientation.w = 1.0
        self.pose_pub.publish(msg)

    def publish_particles(self):
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        for p in self.particles:
            pose = Pose()
            pose.position.x = p[0]
            pose.position.y = p[1]
            pose.position.z = p[2]
            pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.particles_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = BeaconParticleFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
