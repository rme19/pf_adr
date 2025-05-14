import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
import numpy as np

class BeaconParticleFilter(Node):
    def __init__(self):
        super().__init__('particle_filter_node')

        # Parámetros configurables
        self.declare_parameter('num_particles', 5000)
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

        # self.particles = self.initialize_particles()
        self.particles = self.initialize_particles2()
        self.weights = np.ones(self.num_particles) / self.num_particles

        self.current_drone_position = None
        self.current_distance = None

        # Subscripciones
        self.create_subscription(Odometry, '/simple_drone/odom', self.odom_callback, 10)
        self.create_subscription(Float64, '/distance_to_target', self.distance_callback, 10)

        # Publicadores
        self.pose_pub = self.create_publisher(PoseStamped, '/pf/beacon_estimate', 10)
        self.particles_pub = self.create_publisher(PoseArray, '/pf/particles', 10)

        # Temporizador principal
        self.timer = self.create_timer(0.05, self.update_filter)
        self.get_logger().info('Filtro de partículas para estimar baliza inicializado.')

    def initialize_particles(self):
        x = np.random.uniform(self.init_x_range[0], self.init_x_range[1], self.num_particles)
        y = np.random.uniform(self.init_y_range[0], self.init_y_range[1], self.num_particles)
        z = np.random.uniform(self.init_z_range[0], self.init_z_range[1], self.num_particles)
        return np.stack([x, y, z], axis=-1)
    
    def initialize_particles2(self):
        # Inicializar partículas de forma uniforme en una esfera de radio R alrededor del origen      
        phi = np.random.uniform(0, 2 * np.pi, self.num_particles)
        costheta = np.random.uniform(-1, 1, self.num_particles)
        u = np.random.uniform(0, 1, self.num_particles)
        theta = np.arccos(costheta)
        r = self.radius * u**(1/3)  # distribución uniforme en volumen
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

        # Modelo de observación: distancia entre dron y partículas
        diffs = self.particles - self.current_drone_position
        dists = np.linalg.norm(diffs, axis=1)

        self.weights = np.exp(-0.5 * ((dists - self.current_distance) ** 2) / self.sigma**2)
        self.weights += 1e-300
        self.weights /= np.sum(self.weights)

        # Resampleo solo si Neff es bajo (número de partículas efectivas, las que tienen peso significativo)
        if self.effective_sample_size() < self.num_particles / 2:
            self.resample_particles()

        # Publicación
        self.publish_estimate()
        self.publish_particles()

    def effective_sample_size(self):
        return 1.0 / np.sum(np.square(self.weights))

    def resample_particles(self):
        cumulative_sum = np.cumsum(self.weights)
        cumulative_sum[-1] = 1.0  # proteger contra errores numéricos
        indices = np.searchsorted(cumulative_sum, np.random.rand(self.num_particles))
        self.particles = self.particles[indices]

        # Ruido para diversificación
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
