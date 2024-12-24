import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist  # For controlling robot velocity
import math
from nav_msgs.msg import Odometry
import tf_transformations

class LidarAvoidanceNode(Node):

    def __init__(self):
        super().__init__('lidar_avoidance_node')

        # Subscription to the lidar topic
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.lidar_callback,
            10
        )

        # Publisher to the cmd_vel topic for robot movement
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.pose_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.update_pose,
            100
        )

        self.pose = Odometry().pose.pose
        self.rate = self.create_rate(10)  # 10 Hz

    def update_pose(self, data):
        """Callback function to update robot pose."""
        # Get the position and orientation from the Odometry message
        self.pose = data.pose.pose

    def get_robot_yaw(self):
        """Compute the steering angle."""
        # Assuming orientation is in the form of a quaternion
        q = self.pose.orientation
        # Convert quaternion to Euler angles (roll, pitch, yaw)
        roll, pitch, yaw = tf_transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))
        yaw = - yaw
        if yaw < 0:
          return yaw + 2.0*3.14 
        
        return yaw 

    def lidar_callback(self, msg):
        # Process the LaserScan data
        ranges = msg.ranges
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment
        
        # Get the robot's yaw from its pose
        robot_yaw = self.get_robot_yaw()
        robot_yaw=0.0

        # Define the front-facing scan range (±30 degrees around the robot's yaw)
        front_angle_range = 3.14/6.0 #+ 3.14/8.0 # ±30 degrees in radians
        angle_min = max(robot_yaw - front_angle_range, msg.angle_min)
        angle_max = min(robot_yaw + front_angle_range, msg.angle_max)
        
        angle_max_1 = msg.angle_max
        angle_min_1 = (msg.angle_max - front_angle_range)


        # Handle angle wrapping
        #angle_min = max(msg.angle_min, min(angle_min, msg.angle_max))
        #angle_max = max(msg.angle_min, min(angle_max, msg.angle_max))

        # Calculate the start and end indices for the laser scan data
        #start_index = int((angle_min - msg.angle_min) / angle_increment)
        #end_index = int((angle_max - msg.angle_min) / angle_increment)

        start_index = int(angle_min/ angle_increment)
        end_index = int(angle_max/ angle_increment)
        
        start_index_1 = int(angle_min_1/ angle_increment)
        end_index_1 = int(angle_max_1/ angle_increment)-1
        

        # Ensure indices are within bounds
        start_index = max(0, start_index)
        end_index = min(len(ranges) - 1, end_index)

        # Extract front-facing ranges
        front_ranges = ranges[start_index:end_index + 1] + ranges[start_index_1:end_index_1]    
        sign_vel =1.0
        if (max(ranges[start_index:end_index + 1])> max(ranges[start_index_1:end_index_1])):
             sign_vel =1.0
        else:
             sign_vel =-1.0   
        
        # Check if minimum distance in the front is safe for forward movement
        min_distance = min([r for r in front_ranges if not math.isinf(r)], default=float('inf'))
        
        print(robot_yaw, angle_min, angle_max, min_distance)
        safe_distance = 0.6
        max_safe_distance = 2.0
        all_clear = min_distance > safe_distance #and min_distance < max_safe_distance

        # Determine if the robot is "stuck" by checking for repetitive distance values
        stuck = False
        #unique_distances = len(set([round(r, 4) for r in front_ranges if not math.isinf(r)]))
        #if unique_distances <= 2:  # Consider stuck if all values are very similar
        #    stuck = True

        # Create Twist message for robot movement
        move_cmd = Twist()

        if all_clear:
            move_cmd.linear.x = 0.05  # Move forward
            move_cmd.angular.z = 0.0  # No rotation
        else:
            move_cmd.linear.x = 0.0  # Stop forward movement
            move_cmd.angular.z = 0.05  # Rotate to avoid obstacle

        # Publish the movement command if the robot is not "stuck"
        if not stuck:
            self.publisher_.publish(move_cmd)
        else:
            self.get_logger().info("Robot seems stuck; stopping.")
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.0
            self.publisher_.publish(move_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = LidarAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

