import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import random
import math

class PublisherVelocity(Node):

    def __init__(self):
        super().__init__('publisher_velocity')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Twist()
        
        #Completar con el mensaje a enviar por el nodo "publisher_velocity"
        #msg.linear.x = 0.5 + random.uniform(-0.3, 0.3)
        #msg.angular.z = 0.5 + random.uniform(-0.3, 0.3)

        # Movimiento en forma de infinito (lemniscata)
        t = self.i * 0.1  # Parámetro de tiempo
        msg.linear.x = 0.5  # Velocidad lineal constante
        msg.angular.z = 0.8 * math.sin(2 * t)  # Velocidad angular sinusoidal con frecuencia doble
        self.i += 1

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing Twist: linear.x={msg.linear.x}, angular.z={msg.angular.z}')


def main(args=None):
    rclpy.init(args=args)

    publisher_velocity = PublisherVelocity() # inicializa el nodo

    rclpy.spin(publisher_velocity)

    publisher_velocity.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
