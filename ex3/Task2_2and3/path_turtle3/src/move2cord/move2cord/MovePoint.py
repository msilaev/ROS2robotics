#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from turtlesim.msg import Pose
from math import pow, atan2, sqrt
import sys
from nav_msgs.msg import Path
import numpy as np
import time
from std_msgs.msg import Float64

import tf_transformations

from .TurtleNodeClass import TurtleBot

class PointContoller(TurtleBot):

    def __init__(self):
        """Constructor for PointGoalController."""

        super().__init__()
        self.error_publisher = self.create_publisher(Float64, '/distance_error', 2)
        self.distance_error = Float64()

    def set_goals(self, x, y):

        self.goalPose.x = x
        self.goalPose.y = y
        self.get_logger().info(f"New goal set: x={x}, y={y}")

    def move2Goal(self):
        """Moves the turtle to the goal."""

        # Calculate and publish the distance error
        self.distance_error.data = self.euclidean_distance()
        self.error_publisher.publish(self.distance_error)


        goal_pose = Pose()

        goal_pose.x = self.goalPose.x # float(input("Set your x goal: "))
        goal_pose.y = self.goalPose.y #float(input("Set your y goal: "))
        goal_pose.theta = self.goalPose.theta - 2*np.pi*int(self.goalPose.theta/(2*np.pi))

        goal_pose.theta = self.steering_angle()

        distance_tolerance = 0.1

        pose_tolerance = 0.1

        vel_msg = Twist()

        if self.goal_reached == False:

            q = self.pose.pose.pose.orientation

            # Convert quaternion to Euler angles using tf_transformations
            roll, pitch, yaw = tf_transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))
            theta =yaw
            angle_diff = (  self.steering_angle() - theta)

            if (np.abs(angle_diff) > pose_tolerance):

               # Linear velocity in the x-axis.
                vel_msg.linear.x = 0.0
                vel_msg.linear.y = 0.0
                vel_msg.linear.z = 0.0

                # Angular velocity in the z-axis.
                vel_msg.angular.x = 0.0
                vel_msg.angular.y = 0.0
                vel_msg.angular.z = self.angular_vel(constant=1.0)

                # Publishing our vel_msg
                self.velocity_publisher.publish(vel_msg)

                #print(vel_msg)

            elif self.euclidean_distance() >= distance_tolerance:

                #time.sleep(1)
                # Linear velocity in the x-axis.
                vel_msg.linear.x = self.linear_vel(constant=0.5)
                vel_msg.linear.y = 0.0
                vel_msg.linear.z = 0.0

                # Angular velocity in the z-axis.
                vel_msg.angular.x = 0.0
                vel_msg.angular.y = 0.0
                vel_msg.angular.z = 0.0

                self.velocity_publisher.publish(vel_msg)

            else:
                self.goal_reached = True
                print("position goal reached")
                vel_msg.linear.x = 0.0
                vel_msg.angular.z = 0.0
                vel_msg.angular.z = 0.0
                self.velocity_publisher.publish(vel_msg)
                #print("pose goal reached")
                self.get_logger().info("Position goal reached!")

                raise SystemExit

                    #rclpy.close()

def main(args=None):
    rclpy.init(args=args)

    turtlebot_controller = PointContoller()

    if len(sys.argv) < 3:
        print("Usage: ros2 run <package_name> <node_name> <x_goal> <y_goal>")
        return

    x_goal = float(sys.argv[1])
    y_goal = float(sys.argv[2])

    try:
        turtlebot_controller.set_goals(x_goal, y_goal)
        rclpy.spin(turtlebot_controller)
        #print("spin stop")
        #turtlebot_controller.move2goal(x_goal, y_goal, theta_goal)

    except SystemExit:
        pass
        #print("Keyboard Interrupt detected")
        #rclpy.logging.get_logger("Quitting").info('Done')

    finally:
        # Ensure cleanup happens regardless of how the program exits
        turtlebot_controller.destroy_node()
        rclpy.shutdown()
        #print("shutdown complete")  # Will always be printed when exiting


    #print("shotdown 2")

if __name__ == '__main__':
    main()
