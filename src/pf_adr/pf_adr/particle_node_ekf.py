import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose
import numpy as np
import csv
import os
from datetime import datetime
from scipy.stats import normaltest


class BeaconParticleFilter(Node):
    def __init__(self):
        super().__init__('particle_filter_node')

        # Parámetro beacon_id
        self.declare_parameter('beacon_id', 0)
        self.beacon_id = self.get_parameter('beacon_id').value

        # Parámetros configurables
        self.declare_parameter('num_particles', 5000)
        self.declare_parameter('sigma', 0.02)
        self.declare_parameter('noise_std', 0.02)
        self.declare_parameter('radius', 1.0)
        self.declare_parameter('init_x_range', [-1.0, 1.0])
        self.declare_parameter('init_y_range', [-1.0, 1.0])
        self.declare_parameter('init_z_range', [0.0, 1.0])
        self.declare_parameter('noise_pos_drone',[0.003, 0.003, 0.002])
        self.declare_parameter('noise_dist',0.003)
        
        self.num_particles = self.get_parameter('num_particles').value
        self.sigma = self.get_parameter('sigma').value
        self.noise_std = self.get_parameter('noise_std').value
        self.radius = self.get_parameter('radius').value
        self.init_x_range = self.get_parameter('init_x_range').value
        self.init_y_range = self.get_parameter('init_y_range').value
        self.init_z_range = self.get_parameter('init_z_range').value
        self.noise_pos_drone = self.get_parameter('noise_pos_drone').value
        self.noise_dist = self.get_parameter('noise_dist').value

        self.name_fichero = ('archivo_csv_' + str(self.beacon_id))
        # Creación de directorio y archivo CSV
        now = datetime.now().strftime('%Y%m%d_%H%M%S')
        log_dir = os.path.expanduser('~/pf_logs')  # Asegúrate de que este directorio sea correcto
        os.makedirs(log_dir, exist_ok=True)  # Crear la carpeta si no existe
        self.csv_path = os.path.join(log_dir, f'pf_means_{self.beacon_id}.csv')


        # Crear y abrir el archivo CSV
        self.csv_file = open(self.csv_path, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp', 'x_mean', 'y_mean', 'z_mean'])


        self.particles = self.initialize_particles2()
        self.weights = np.ones(self.num_particles) / self.num_particles

        self.current_drone_position = None
        self.current_distance = None

        self.gaussian = False  # Variable para controlar la gaussianidad

        # Subscripciones
        self.create_subscription(Odometry, '/simple_drone/odom', self.odom_callback, 10)
        self.create_subscription(Float64, f'/beacon_{self.beacon_id}/distance_to_target', self.distance_callback, 10)
        # Publicador del estado estimado
        self.ekf_pub = self.create_publisher(PoseWithCovarianceStamped, f'/beacon_{self.beacon_id}/pf_beacon_init', 10)

        # Publicadores con topics específicos
        self.pose_pub = self.create_publisher(PoseStamped, f'/pf/beacon_{self.beacon_id}/estimate', 10)
        self.particles_pub = self.create_publisher(PoseArray, f'/pf/beacon_{self.beacon_id}/particles', 10)

        self.timer = self.create_timer(0.05, self.update_filter)
        self.get_logger().info(f'Filtro de partículas para baliza {self.beacon_id} inicializado.')

        self.start_time = self.get_clock().now()

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
        noise_drone = np.random.normal(0, self.noise_pos_drone, 3)
        self.current_drone_position = np.array([
            msg.pose.pose.position.x + noise_drone[0],
            msg.pose.pose.position.y + noise_drone[1],
            msg.pose.pose.position.z + noise_drone[2]
        ])

    def distance_callback(self, msg):
        if msg.data < 0.0:
            self.current_distance = None
    
        else:
            noise_distance = np.random.normal(0, self.noise_dist)
            self.current_distance = msg.data + noise_distance

    def update_filter(self):
        if self.current_drone_position is None or self.current_distance is None or self.gaussian is True:
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

        # # Check gaussianity every second
        if int((self.get_clock().now() - self.start_time).nanoseconds * 1e-9) % 2 == 0:
            # self.get_logger().info("Error de distancia aceptable, verificando gaussianidad.")
            self.check_gaussianity()

    def check_gaussianity(self):
        # Obtener las medias y covarianzas ponderadas
        mean = np.average(self.particles, axis=0, weights=self.weights)
        cov = np.cov(self.particles.T, aweights=self.weights)

        # Test de normalidad de D’Agostino y Pearson por dimensión
        results = []
        for i in range(3):  # Para x, y, z
            stat, p = normaltest(self.particles[:, i], axis=0)
            results.append((stat, p))

        # self.get_logger().info("Test de normalidad por dimensión (stat, p): " + str(results))
        # self.get_logger().info("P valor: " + str(p))
        # Puedes definir un umbral de p-valor típico (ej. 0.05)
        all_gaussian = all(p > 0.01 for (_, p) in results)
        if all_gaussian:
            self.get_logger().info("⚠️ La distribución de partículas parece gaussiana.")
            
            self.msg_ekf = PoseWithCovarianceStamped()
            self.msg_ekf.header.stamp = self.get_clock().now().to_msg()
            self.msg_ekf.header.frame_id = 'map'

            # Asignar la posición media
            self.msg_ekf.pose.pose.position.x = mean[0]
            self.msg_ekf.pose.pose.position.y = mean[1]
            self.msg_ekf.pose.pose.position.z = mean[2]

            # Asignar orientación por defecto (no estimamos orientación aquí)
            self.msg_ekf.pose.pose.orientation.w = 1.0

            # Construir matriz de covarianza 6x6
            cov_6x6 = np.zeros((6, 6))
            cov_6x6[0:3, 0:3] = cov  # Solo usamos las 3 primeras dimensiones

            # Convertir a lista de 36 elementos (orden fila mayor)
            self.msg_ekf.pose.covariance = cov_6x6.flatten().tolist()

            self.ekf_pub.publish(self.msg_ekf)
            self.get_logger().info(f'✅ La distribución de la baliza {self.beacon_id} es gaussiana. Publicando estado EKF.')
            self.get_logger().info("Posición estimada: " + str(mean) + " Covarianza: " + str(cov))

            self.gaussian = True
        else:
            pass
            # self.get_logger().info("❌ La distribución NO es gaussiana. No conviene usar EKF todavía.")

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

        # Guardar en el CSV
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        self.csv_writer.writerow([elapsed_time, mean[0], mean[1], mean[2]])
        self.csv_file.flush()

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
    node.csv_file.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()