import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Int32MultiArray
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
import numpy as np
import csv
import os
from datetime import datetime
from scipy.stats import normaltest
import time


class BeaconParticleFilter(Node):
    def __init__(self):
        super().__init__('particle_filter_node')

        # Parámetro beacon_id
        self.declare_parameter('beacon_id', 0)
        self.beacon_id = self.get_parameter('beacon_id').value

        self.use_ekf = False
        self.ekf_started = False
        self.ekf_start_time = None
        self.ekf_times_path = os.path.expanduser('~/pf_logs/ekf_start_times.csv')

        self.normality_threshold = 0.2  # nuevo umbral más estricto

        # Parámetros globales
        self.declare_parameter('total_num_particles', 5000)
        self.total_num_particles = self.get_parameter('total_num_particles').value

        # Otros parámetros
        self.declare_parameter('sigma', 0.2)
        self.declare_parameter('noise_std', 0.2)
        self.declare_parameter('radius', 1.0)

        self.sigma = self.get_parameter('sigma').value
        self.noise_std = self.get_parameter('noise_std').value
        self.radius = self.get_parameter('radius').value

        self.declare_parameter('noise_pos_drone', [0.003, 0.003, 0.002])
        self.declare_parameter('noise_dist', 0.003)
        self.noise_pos_drone = self.get_parameter('noise_pos_drone').value
        self.noise_dist = self.get_parameter('noise_dist').value

        self.current_drone_position = None
        self.current_distance = None

        self.name_fichero = ('archivo_csv_' + str(self.beacon_id))

        # Creación de directorio y archivo CSV
        now = datetime.now().strftime('%Y%m%d_%H%M%S')
        log_dir = os.path.expanduser('~/pf_logs')
        os.makedirs(log_dir, exist_ok=True)
        self.csv_path = os.path.join(log_dir, f'pf_means_{self.beacon_id}.csv')

        self.csv_file = open(self.csv_path, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp', 'x_mean', 'y_mean', 'z_mean'])

        self.start_time = self.get_clock().now()
        self.start_time_wall = time.time()  # Tiempo de inicio en reloj de pared

        self.num_particles = 0
        self.particles = np.zeros((0, 3))
        self.weights = np.zeros(0)
        self.estimated_position = np.zeros(3)  # Inicializa posición estimada

        self.create_subscription(Odometry, '/simple_drone/odom', self.odom_callback, 10)
        self.create_subscription(Float64, f'/beacon_{self.beacon_id}/distance_to_target', self.distance_callback, 10)
        self.create_subscription(Int32MultiArray, '/beacon_particle_distribution', self.particle_distribution_callback, 10)

        self.pose_pub = self.create_publisher(PoseStamped, f'/pf/beacon_{self.beacon_id}/estimate', 10)
        self.particles_pub = self.create_publisher(PoseArray, f'/pf/beacon_{self.beacon_id}/particles', 10)

        self.timer = self.create_timer(0.05, self.update_filter)

        self.get_logger().info(f'Filtro de partículas para baliza {self.beacon_id} inicializado.')

    def initialize_particles(self, num):
        if num == 0:
            return np.zeros((0, 3))
        phi = np.random.uniform(0, 2 * np.pi, num)
        costheta = np.random.uniform(-1, 1, num)
        u = np.random.uniform(0, 1, num)
        theta = np.arccos(costheta)
        r = self.radius * u**(1/3)
        x = r * np.sin(theta) * np.cos(phi)
        y = r * np.sin(theta) * np.sin(phi)
        z = r * costheta
        return np.stack([x, y, z], axis=-1)

    def initialize_particles_around_point(self, count, center, radius=0.5):
        offsets = np.random.normal(0, radius, size=(count, 3))
        return np.array(center) + offsets

    def check_particles_are_gaussian(self):
        if self.particles.shape[0] < 8:
            self.get_logger().info("No hay suficientes partículas para evaluar.")
            return False
        passed = []
        for i in range(3):
            try:
                values = self.particles[:, i]
                repeated_values = np.repeat(values, (self.weights * 1000).astype(int))
                if len(repeated_values) >= 8:
                    stat, p = normaltest(repeated_values)
                    self.get_logger().info(f'Dim {i} - p={p:.4f}, muestras={len(repeated_values)}')
                    passed.append(p > self.normality_threshold)
                else:
                    self.get_logger().warn(f'Dim {i} - No suficientes muestras para normaltest: {len(repeated_values)}')
                    passed.append(False)
            except Exception as e:
                self.get_logger().warn(f'Error al evaluar normalidad en dimensión {i}: {e}')
                passed.append(False)
        return all(passed)

    def initialize_ekf(self):
        self.ekf_state = np.average(self.particles, axis=0, weights=self.weights)
        self.ekf_cov = np.cov(self.particles.T, aweights=self.weights)

    def ekf_update(self):
        x = self.ekf_state
        P = self.ekf_cov
        z = self.current_distance

        dx = x - self.current_drone_position
        dist = np.linalg.norm(dx)
        if dist < 1e-6:
            return
        H = (dx / dist).reshape(1, 3)
        R = np.array([[self.sigma**2]])
        S = H @ P @ H.T + R
        K = P @ H.T @ np.linalg.inv(S)
        y = np.array([[z - dist]])
        self.ekf_state = x + (K @ y).flatten()
        self.ekf_cov = (np.eye(3) - K @ H) @ P

    def publish_ekf_estimate(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = self.ekf_state
        msg.pose.orientation.w = 1.0
        self.pose_pub.publish(msg)

        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        self.csv_writer.writerow([elapsed_time, *self.ekf_state])
        self.csv_file.flush()

        self.estimated_position = self.ekf_state

    def particle_distribution_callback(self, msg):
        if self.beacon_id >= len(msg.data):
            self.get_logger().warn(
                f'Se recibió distribución con longitud {len(msg.data)}, pero beacon_id={self.beacon_id} está fuera de rango.'
            )
            return

        new_count = msg.data[self.beacon_id]
        if new_count != self.num_particles:
            self.num_particles = new_count
            if new_count == 0:
                self.particles = np.zeros((0, 3))
                self.weights = np.zeros(0)
                self.get_logger().info(f'Baliza {self.beacon_id} inactiva. Filtro desactivado.')
            else:
                if hasattr(self, 'estimated_position'):
                    center = self.estimated_position
                else:
                    center = np.zeros(3)
                    self.get_logger().warn(f'No hay estimación previa. Inicializando partículas en el origen.')
                self.particles = self.initialize_particles_around_point(new_count, center)
                self.weights = np.ones(new_count) / new_count
                self.get_logger().info(f'Baliza {self.beacon_id} activa. {new_count} partículas asignadas en torno a {center}.')

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
        # Si no hay datos suficientes, escribir la última estimación guardada
        if self.current_drone_position is None or self.current_distance is None:
            # No se puede estimar nada
            return

        if self.use_ekf:
            # Actualizar EKF y publicar
            self.ekf_update()
            self.publish_ekf_estimate()
            return

        if self.num_particles == 0:
            # No hay partículas, escribir última estimación conocida
            elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
            self.csv_writer.writerow([elapsed_time, *self.estimated_position])
            self.csv_file.flush()
            # También publicar la última estimación
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = self.estimated_position
            msg.pose.orientation.w = 1.0
            self.pose_pub.publish(msg)
            return

        # Actualizar pesos del filtro de partículas
        diffs = self.particles - self.current_drone_position
        dists = np.linalg.norm(diffs, axis=1)

        self.weights = np.exp(-0.5 * ((dists - self.current_distance) ** 2) / self.sigma**2)
        self.weights += 1e-300
        self.weights /= np.sum(self.weights)

        if self.effective_sample_size() < self.num_particles / 2:
            self.resample_particles()

        self.publish_estimate()
        self.publish_particles()

        if self.check_particles_are_gaussian():
            if not self.ekf_started:
                self.get_logger().info("Partículas gaussianas detectadas. Cambiando a EKF.")
                self.initialize_ekf()
                self.use_ekf = True
                self.ekf_started = True
                self.ekf_start_time = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
                self.save_ekf_start_time()
        else:
            self.get_logger().info("Partículas NO gaussianas todavía.")

    def effective_sample_size(self):
        return 1.0 / np.sum(np.square(self.weights)) if self.num_particles > 0 else 0

    def resample_particles(self):
        cumulative_sum = np.cumsum(self.weights)
        cumulative_sum[-1] = 1.0
        indices = np.searchsorted(cumulative_sum, np.random.rand(self.num_particles))
        self.particles = self.particles[indices]
        noise = np.random.normal(0, self.noise_std, self.particles.shape)
        self.particles += noise
        self.weights.fill(1.0 / self.num_particles)

    def save_ekf_start_time(self):
        if not os.path.exists(self.ekf_times_path):
            with open(self.ekf_times_path, mode='w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['beacon_id', 'ekf_start_time'])

        existing = []
        try:
            with open(self.ekf_times_path, mode='r') as f:
                reader = csv.reader(f)
                existing = list(reader)
        except Exception:
            pass

        with open(self.ekf_times_path, mode='a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([self.beacon_id, self.ekf_start_time])

    def publish_estimate(self):
        mean = np.average(self.particles, axis=0, weights=self.weights)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = mean
        msg.pose.orientation.w = 1.0
        self.pose_pub.publish(msg)

        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        self.csv_writer.writerow([elapsed_time, *mean])
        self.csv_file.flush()

        self.estimated_position = mean

    def publish_particles(self):
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        if self.num_particles == 0 or self.particles.size == 0:
            self.particles_pub.publish(msg)
            return

        for p in self.particles:
            pose = Pose()
            pose.position.x = p[0]
            pose.position.y = p[1]
            pose.position.z = p[2]
            pose.orientation.w = 1.0
            msg.poses.append(pose)

        self.particles_pub.publish(msg)

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BeaconParticleFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
