import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

class KeyboardSubscriber(Node):

    def __init__(self):
        super().__init__('keyboard_subscriber')
        self.subscription = self.create_subscription(
            Int32,
            'keyboard/keypress',
            self.movement_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

    def movement_callback(self, msg):
        print(msg)
        print(msg.data)
        key_number = int(msg.data)
        move_cmd = Twist()
        if key_number == 87:  # w key
            move_cmd.linear.x = 1.0
            move_cmd.linear.z = 0.0
        if key_number == 65:  # a key
            move_cmd.angular.z = 0.5
            move_cmd.linear.x = 0.0
        if key_number == 83:  # s key 
            move_cmd.linear.x = -1.0
            move_cmd.linear.z = 0.0
        if key_number == 68:  # d keyd
            move_cmd.angular.z = -0.5
            move_cmd.linear.x = 0.0
        self.publisher_.publish(move_cmd)

def main(args=None):
    rclpy.init(args=args)

    keyboard_subscriber = KeyboardSubscriber()

    rclpy.spin(keyboard_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    keyboard_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()