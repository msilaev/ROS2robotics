#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sympy.abc import theta
from turtlesim.msg import Pose
from math import pow, atan2, sqrt
import sys
from nav_msgs.msg import Path
import numpy as np
import time

import tf_transformations
from std_msgs.msg import Float64

from .TurtleNodeClass import TurtleBot

class PathContoller(TurtleBot):

    def __init__(self):
        """Constructor for PointGoalController."""
        super().__init__()  # Call the constructor of the parent class
        # Any other specific initialization for PointGoalController can be added here
        self.path = []

        self.error_publisher = self.create_publisher(Float64, '/distance_error', 2)
        self.distance_error = Float64()


    def set_goals(self, path):

        self.path = path

        self.get_logger().info(f"New goal set: path of {len(path)} points")

    def move2Goal(self):
        """Moves the turtle to the goal."""

        #goal_pose = Pose()

        # Calculate and publish the distance error
        self.distance_error.data = self.euclidean_distance()
        self.error_publisher.publish(self.distance_error)

        self.goalPose.x = float(self.path[ len(self.path)- self.vertInd -1 ][0])  # float(input("Set your x goal: "))
        self.goalPose.y = float(self.path[ len(self.path) - self.vertInd -1][1])  # float(input("Set your y goal: "))
        self.goalPose.theta = 0.0

        #goal_pose.theta = self.steering_angle()

        distance_tolerance = 0.1
        pose_tolerance = 0.1
        vel_msg = Twist()

        q = self.pose.pose.pose.orientation

        # Convert quaternion to Euler angles using tf_transformations
        roll, pitch, yaw = tf_transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))

        theta = yaw

        angle_diff = self.steering_angle() - theta

        #print(self.euclidean_distance())

        if np.abs(angle_diff) > pose_tolerance:

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

                #print(np.abs(angle_diff), theta)

        elif self.euclidean_distance() >= distance_tolerance:

                #time.sleep(1)

                # Linear velocity in the x-axis.
                vel_msg.linear.x = self.linear_vel(constant=1)
                vel_msg.linear.y = 0.0
                vel_msg.linear.z = 0.0

                # Angular velocity in the z-axis.
                vel_msg.angular.x = 0.0
                vel_msg.angular.y = 0.0
                vel_msg.angular.z = 0.0

                self.velocity_publisher.publish(vel_msg)

                q = self.pose.pose.pose.orientation

                # Convert quaternion to Euler angles using tf_transformations
                roll, pitch, yaw = tf_transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))

                theta = yaw

        elif self.vertInd < len(self.path)-1:

            #self.goal_reached = True
            if (self.vertInd%1 ==0):
                print("position goal reached", self.vertInd, self.path[ len(self.path)- self.vertInd -1])

            self.vertInd +=1

        else :

            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
            vel_msg.angular.z = 0.0
            self.velocity_publisher.publish(vel_msg)
            print("Final goal reached")
            self.get_logger().info("Path end reached!")
            raise SystemExit

        #rclpy.close()

def main(args=None):

    rclpy.init(args=args)
    path = []

    choice = input(
        "--Press 1 if you want to follow square path\n--Press 2 if you want to follow backwards recorded path\n")

    if choice == "1":
        filename= 'robot_square.log'
    elif choice == "2":
        filename = "robot_footsteps.log"
    else:
        raise ValueError("Invalid choice. Please press 1 or 2.")

    with open(filename, "r") as f:

       for line in f:

            #print(line)

            s = line.split(",")
            path.append( (s[0], s[1]))

    turtlebot_controller = PathContoller()
    turtlebot_controller.set_goals(path)

    try:
    #        turtlebot_controller.set_goals(x_goal, y_goal, theta_goal)

        rclpy.spin(turtlebot_controller)

    except SystemExit:
            pass
        #print("Keyboard Interrupt detected")
        #rclpy.logging.get_logger("Quitting").info('Done')

    #finally:
        # Ensure cleanup happens regardless of how the program exits
    turtlebot_controller.destroy_node()
    rclpy.shutdown()
        #print("shutdown complete")  # Will always be printed when exiting


    #print("shotdown 2")

if __name__ == '__main__':
    main()
