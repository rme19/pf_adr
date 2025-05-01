import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
import numpy as np

class BeaconParticleFilter(Node):
    def __init__(self):
        super().__init__('beacon_particle_filter')

        # Parámetros configurables
        self.declare_parameter('num_particles', 1000)
        self.num_particles = self.get_parameter('num_particles').value
        self.declare_parameter('sigma', 0.02)  # Desviación estándar para la medición de distancia
        self.sigma = self.get_parameter('sigma').value

        self.particles = self.initialize_particles()
        self.weights = np.ones(self.num_particles) / self.num_particles

        self.current_drone_position = None
        self.current_distance = None

        # Subscripciones
        self.create_subscription(Odometry, '/simple_drone/odom', self.odom_callback, 10)
        self.create_subscription(Float64, '/distance_to_target', self.distance_callback, 10)

        # Publicadores para RViz
        self.pose_pub = self.create_publisher(PoseStamped, '/pf/beacon_estimate', 10)
        self.particles_pub = self.create_publisher(PoseArray, '/pf/particles', 10)

        self.timer = self.create_timer(0.1, self.update_filter)
        self.get_logger().info('Filtro de partículas para estimar baliza inicializado.')

    def initialize_particles(self):
        # Inicializar partículas alrededor de la posición esperada (en este caso, la baliza está en 0,0,0)
        x = np.random.uniform(-1, 1, self.num_particles)
        y = np.random.uniform(-1, 1, self.num_particles)
        z = np.random.uniform(0, 1, self.num_particles)
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

        # Calcular distancias desde cada partícula al dron
        diffs = self.particles - self.current_drone_position
        dists = np.linalg.norm(diffs, axis=1)

        # Calcular pesos con un modelo Gaussiano (ruido en distancia)
        self.weights = np.exp(-0.5 * ((dists - self.current_distance) ** 2) / self.sigma**2)
        self.weights += 1e-300  # evitar ceros
        self.weights /= np.sum(self.weights)

        # Resamplear partículas de forma eficiente
        self.resample_particles()

        # Publicar estimación y partículas
        self.publish_estimate()
        self.publish_particles()

    def resample_particles(self):
        # Resampling sistemático para mejorar la precisión
        cumulative_sum = np.cumsum(self.weights)
        cumulative_sum[-1] = 1.0  # Asegurarse de que no haya desbordamientos
        indices = np.searchsorted(cumulative_sum, np.random.rand(self.num_particles))
        self.particles = self.particles[indices]

         # Añadir ruido Gaussiano para evitar colapso del filtro
        noise_std = 0.01  # Puedes ajustar este valor
        noise = np.random.normal(0, noise_std, self.particles.shape)
        self.particles += noise

        self.weights.fill(1.0 / self.num_particles)  # Pesos uniformes después del resampling

    def publish_estimate(self):
        # Estimación de la posición basada en las partículas
        mean = np.average(self.particles, axis=0, weights=self.weights)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = mean[0]
        msg.pose.position.y = mean[1]
        msg.pose.position.z = mean[2]
        msg.pose.orientation.w = 1.0  # sin orientación
        self.pose_pub.publish(msg)

    def publish_particles(self):
        # Publicar todas las partículas en RViz
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
