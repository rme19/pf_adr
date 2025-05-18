import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import math
import random


class DistanceToTargetNode(Node):
    def __init__(self):
        super().__init__('beacon_node')

        # Obtener beacon_id desde parámetros
        self.declare_parameter('beacon_id', 0)
        self.beacon_id = self.get_parameter('beacon_id').value

        # Obtener la posición objetivo desde parámetros
        self.declare_parameter('target_position', [0.0, 0.0, 0.0])
        self.target_position = self.get_parameter('target_position').value
    
        # Rango fijo de intervalos aleatorios
        self.outlier_interval_min = 90.0          
        self.outlier_interval_max = 120.0
        self.interval = 100.0

        self.message_counter = 0.0

        # Suscribirse a la odometría del dron
        self.odom_sub = self.create_subscription(Odometry, '/simple_drone/odom', self.odom_callback, 10)

        # Crear publisher personalizado por ID de baliza
        topic_name = f'/beacon_{self.beacon_id}/distance_to_target'
        self.distance_pub = self.create_publisher(Float64, topic_name, 10)


    def odom_callback(self, msg):
        # Obtener la posición del dron desde el mensaje de Odometry
        drone_position = msg.pose.pose.position
        drone_x = drone_position.x
        drone_y = drone_position.y
        drone_z = drone_position.z

        # Calcular la distancia Euclidiana entre la posición definida y la posición del dron
        distance = math.sqrt(
            (drone_x - self.target_position[0]) ** 2 +
            (drone_y - self.target_position[1]) ** 2 +
            (drone_z - self.target_position[2]) ** 2
        )
        # Comprobar si la distancia esta dentro del limite de deteccion
        if distance > 6.0:
            distance = -1.0       
        
        else:
            if self.message_counter >= self.interval:
                self.get_logger().warn(f'Injectando OUTLIER en el mensaje {self.message_counter}')
                distance = -1.0
                self.message_counter = 0.0
                self.interval = random.uniform(self.outlier_interval_min, self.outlier_interval_max)
            
            else: 
                self.message_counter += 1.0
                
        # Publicar la distancia
        distance_msg = Float64()
        distance_msg.data = distance
        self.distance_pub.publish(distance_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DistanceToTargetNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()