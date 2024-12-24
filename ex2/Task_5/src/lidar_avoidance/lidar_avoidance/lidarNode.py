import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist  # For controlling robot velocity

class LidarAvoidanceNode(Node):
    def __init__(self):
        super().__init__('lidar_avoidance_node')

        # Subscription to the lidar topic
        self.subscription = self.create_subscription(
            LaserScan,
            'lidar',
            self.lidar_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Publisher to the cmd_vel topic for robot movement
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        self.count =0

        #self.rotation_time = 0.1
        #self.rotation_velocity = 3.14/(4*self.rotation_time)

    def lidar_callback(self, msg):
        # Process the LaserScan data
        ranges = msg.ranges
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        front_angle_range = 0.5

        front_angle = (angle_max + angle_min)/2
        angle_increment = msg.angle_increment

        start_index = int((front_angle - front_angle_range - angle_min)/angle_increment)
        end_index = int((front_angle + front_angle_range - angle_min) / angle_increment)
        front_ranges = ranges[start_index:end_index+1]

        #self.get_logger().info(f'Lidar range data: {ranges[:5]}')  # Log first 5 ranges

        # Check if all Lidar readings are above a threshold of 1 meter
        all_more = all(r >= 0.50 for r in front_ranges)

        stuck = False
        all_more = min(front_ranges) > 2.0 and min(front_ranges) < 10

        #print(min(front_ranges), all_more)

        first_numbers = [round(r,4) for r in front_ranges if not math.isinf(r)]
        #print(first_numbers)
        if len(first_numbers) > 4:
            comparison_values = []
            for idx in range(0, len(first_numbers)-1):
                if math.isclose(first_numbers[idx], first_numbers[idx+1],rel_tol=0.00059):
                    comparison_values.append(True)
                else:
                    comparison_values.append(False)
            if all(x == True for x in comparison_values):    
                stuck = True
            else:
                comparison_values = []
    
        
        #print(front_ranges)

        # Create Twist message for robot movement
        move_cmd = Twist()

        if all_more:
            self.count =0
            move_cmd.linear.x = 1.0
            move_cmd.linear.z = 0.0
            # If all distances are greater than 1 meter, move forward
            #move_cmd.linear.x = 1.0  # Move forward with speed 0.5 m/s
            #move_cmd.angular.z = 0.0  # No rotation
        else:

            #self.count +=1

            # If any distance is less than 1 meter, rotate
            #move_cmd.linear.x = 0.0  # Stop forward movement

            #if (self.count<100):
            move_cmd.angular.z = 1.0*0.5  # Rotate with angular velocity
            move_cmd.linear.x = 0.0
            #else:
            #    move_cmd.angular.z = 0.0
            #    move_cmd.linear.x = 2.0

                # Publish the Twist message to control the robot's movement
        if not stuck:
            self.publisher_.publish(move_cmd)
        else:
            raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = LidarAvoidanceNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info('Done')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
