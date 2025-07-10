import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Twist

class TwistStampedToTwist(Node):
    def __init__(self):
        super().__init__('twist_stamped_to_twist')

        # Suscriptor al tópico /cmd_vel con mensajes TwistStamped
        self.subscription = self.create_subscription(
            TwistStamped,
            '/cmd_vel',
            self.listener_callback,
            10)

        # Publicador en otro tópico /cmd_vel_twist de tipo Twist
        self.publisher = self.create_publisher(Twist, '/cmd_vel_twist', 10)

    def listener_callback(self, msg: TwistStamped):
        # Extraemos el Twist y lo publicamos en el nuevo tópico
        twist_msg = Twist()
        twist_msg.linear = msg.twist.linear
        twist_msg.angular = msg.twist.angular

        self.publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TwistStampedToTwist()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
