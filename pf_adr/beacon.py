import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import math

class DistanceToTargetNode(Node):
    def __init__(self):
        super().__init__('distance_to_target_node')

        # Parámetros: posición definida
        self.declare_parameter('target_position', [0.0, 0.0, 0.0])  # (x, y, z)
        self.target_position = self.get_parameter('target_position').value

        # Suscripción al topic /simple_drone/odom
        self.odom_sub = self.create_subscription(Odometry, '/simple_drone/odom', self.odom_callback, 10)

        # Publicación de la distancia en /distance_to_target
        self.distance_pub = self.create_publisher(Float64, '/distance_to_target', 10)

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
