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

from abc import ABC, abstractmethod

class TurtleBot(Node, ABC):

    def __init__(self):
        # Initialize the node
        super().__init__('turtlebot_controller')

        self.goal_reached = False
        self.vertInd = 0
        # Publisher to publish on '/turtle1/cmd_vel'
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Publisher for the path
        self.path_publisher = self.create_publisher(Path, '/turtle1/path', 10)

        self.pose_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.update_pose,
            100
        )

        self.pose = Pose()
        self.goalPose = Pose()
        self.rate = self.create_rate(10)  # 10 Hz


    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is received."""
        self.pose = data
        self.move2Goal()

    def euclidean_distance(self):
        """Euclidean distance between current pose and the goal."""

        goal_pose= self.goalPose
        return sqrt(pow((goal_pose.x - self.pose.pose.pose.position.x), 2) +
                    pow((goal_pose.y - self.pose.pose.pose.position.y), 2))

    def linear_vel(self, constant=1.0):
        """Compute the linear velocity."""

        return constant * self.euclidean_distance()

    def steering_angle(self, theta = None):
        """Compute the steering angle."""

        # Assuming orientation is in the form of a quaternion
        q = self.pose.pose.pose.orientation
        # Convert quaternion to Euler angles (roll, pitch, yaw)
        roll, pitch, yaw = tf_transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))

        if theta is None:

            return atan2(self.goalPose.y - self.pose.pose.pose.position.y,
                         self.goalPose.x - self.pose.pose.pose.position.x)
        else:
            return - yaw

    def angular_vel(self, constant=1.0, th = None):
        """Compute the angular velocity."""

        q = self.pose.pose.pose.orientation
        # Convert quaternion to Euler angles (roll, pitch, yaw)
        roll, pitch, yaw = tf_transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))

        if th is None:

            dy =  self.goalPose.y - self.pose.pose.pose.position.y
            dx =  self.goalPose.x - self.pose.pose.pose.position.x

            return constant * ( dy * np.cos(yaw) - dx * np.sin(yaw) )/ ( dx**2 + dy **2 )**(0.5)
            # return constant * ( self.steering_angle() - yaw)

        else:
            return constant*th


    @abstractmethod
    def set_goals(self):
        """Abstract method to set goals, to be defined in child classes."""
        pass

    @abstractmethod
    def move2Goal(self):
        """Abstract method to move to the goal, to be defined in child classes."""
        pass

