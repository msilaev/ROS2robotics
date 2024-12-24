import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdometryLogger(Node):

    def __init__(self):
        super().__init__('odometry_logger')
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 2)
        self.file = open('robot_footsteps.log', 'w')

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        self.file.write(f"{position.x}, {position.y}, {position.z}\n")
        self.get_logger().info(f"Logged position: {position.x}, {position.y}, {position.z}")

    def destroy_node(self):
        self.file.close()
        super().destroy_node()

def main(args=None):

    print("Please move robot")

    rclpy.init(args=args)

    node = OdometryLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
