# Copyright (C) 2021 Bosch LLC CR, North America. All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

# TODO: This script should allow the user to set an array of values in the terminal representing the desired pose target. -> 7 values: x, y, z, r, p, yaw, open/close
# TODO: Create a runner script like you do in the panda_ros2_gazebo repo to allow the user to choose between the two setpoint selection methods (teleop or command line)

import sys
import numpy as np
from scipy.spatial.transform import Rotation as R
import copy
from typing import List

# ROS2 Python API libraries
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.client import Client
from rclpy.qos import qos_profile_system_default

# ROS2 message and service data structures
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty

# Helpers
from .helpers import quat2rpy, rpy2quat

# Configure numpy output
np.set_printoptions(precision=4, suppress=True)

def execute(args):
    rclpy.init(args=args)

    app = PandaSetpoint()
    app.poll_terminal()

    app.destroy_node()
    rclpy.shutdown()

class PandaSetpoint(Node):
    def __init__(self):
        super().__init__('panda_teleop_control')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('base_frame', None),
                ('end_effector_frame', None),
                ('end_effector_target_topic', None),
                ('end_effector_pose_topic', None)
            ]
        )

        # TODO: At the moment I'm unable to load the parameter directly from the parameter server. Later I need to modify this code to include that functionality.
        # Create end effector target publisher
        self._end_effector_target_publisher: Publisher = self.create_publisher(Odometry, 'end_effector_target_pose', qos_profile_system_default)
        self._end_effector_pose_subscriber: Subscription = self.create_subscription(Odometry, '/end_effector_pose', self.callback_end_effector_odom, 10)

        # Create a service for actuating the gripper. The service is requested via teleop
        self._actuate_gripper_client: Client = self.create_client(Empty, 'actuate_gripper')

        # The initial pose is just the end effector location in the base frame at the nominal joint angles
        self._end_effector_target_origin: Odometry = Odometry()
        self._end_effector_target_origin.pose.pose.position.x = 0.30701957005161057
        self._end_effector_target_origin.pose.pose.position.y = -5.934817164959582e-12
        self._end_effector_target_origin.pose.pose.position.z = 0.4872695582766443
        self._end_effector_target_origin.pose.pose.orientation.x = -0.00014170976139083377
        self._end_effector_target_origin.pose.pose.orientation.y = 0.7071045301233027
        self._end_effector_target_origin.pose.pose.orientation.z = 0.00014171064119222223
        self._end_effector_target_origin.pose.pose.orientation.w = 0.7071090038427887
        self._end_effector_target_origin.header.frame_id = 'panda_link0' # TODO: Remove hardcoded parameters
        self._end_effector_target_origin.child_frame_id = 'end_effector_frame' # TODO: Remove hardcoded parameters
        self._end_effector_target_origin.header.stamp = self.get_clock().now().to_msg()

        self._end_effector_target: Odometry = copy.deepcopy(self._end_effector_target_origin)
        self._end_effector_pose: Odometry = copy.deepcopy(self._end_effector_target)

        # publish the initial end effector target, which corresponds to the joints at their neutral position
        self._end_effector_target_publisher.publish(self._end_effector_target)

        self.MSG_TERMINAL = """
Enter an end effector pose target:
- The target should contain 7 values:
   - x, y, z target in CENTIMETERS
   - r, p, yaw target (x-y-z Euler angles) in DEGREES
   - change the gripper state (1: open->close or close->open, 0: keep current state)
        """

        self.MSG_POSE = """CURRENT END EFFECTOR TARGET POSE:
[x, y, z] = [{}, {}, {}] m
[r, p, y] = [{}, {}, {}] ° (Euler)
CURRENT END EFFECTOR POSE:
[x, y, z] = [{}, {}, {}] m
[r, p, y] = [{}, {}, {}] ° (Euler)"""

        self._translation_limits = [[0.0, 1.0], [-1.0, 1.0], [0.0, 1.0]] # xyz
        self._rotation_limits = [[-90., 90.], [-90., 90.], [-90., 90.]] # rpy

    def callback_end_effector_odom(self, odom: Odometry):
        self._end_effector_pose = odom

    def _publish(self):

        self._end_effector_target_publisher.publish(self._end_effector_target)

    def _parse_user_input(self, user_input):

        if len(user_input) != 7:
            print("[panda_teleop]: [ERR] Setpoint should contain 7 values.")
            return False

        return True

    def _set_pose_target(self, user_input):

        # TODO: Make sure to convert everything to a float before putting it into the message structure
        for i, value in enumerate(user_input):
            if i < 3:
                # set the translation target
                self._end_effector_target.pose.pose.position.x = np.clip(np.float_(value), self._translation_limits[0][0], self._translation_limits[0][1])

                self._end_effector_target.pose.pose.position.y = np.clip(np.float_(value), self._translation_limits[1][0], self._translation_limits[1][1])

                self._end_effector_target.pose.pose.position.z = np.clip(np.float_(value), self._translation_limits[2][0], self._translation_limits[2][1])

            if i >= 3 and i < 6:
                # set the rotation target
                euler_target = [0., 0., 0.]
                for j in range(3):
                    euler_target[j] = np.clip(np.float_(value), self._rotation_limits[j][0], self._rotation_limits[j][1])

                self._end_effector_target.pose.pose.orientation = copy.deepcopy(rpy2quat(euler_target, input_in_degrees=True))

            if i == 6:
                if np.float_(value) > 0:
                    # Call the service to actuate the gripper
                    future = self._actuate_gripper_client.call_async(Empty.Request())
                    if future.done():
                        try:
                            response = future.result()
                        except Exception as e:
                            self.get_logger().info('SERVICE CALL TO ACTUATE GRIPPER SERVICE FAILED %r' % (e,))
                        else:
                            self.get_logger().info('GRIPPER ACTUATED SUCCESSFULLY')

    def poll_terminal(self):

        try:
            while(1):
                setpoint = input("[x, y, z, r, p, yaw, gripper] = ")

                if self._parse_user_input(setpoint):
                    self._set_pose_target(setpoint)

                    print(self.MSG_TERMINAL)

                    rclpy.spin_once(self) # this is necessary, otherwise the odom callback will never be called within this while loop

                    euler_current = quat2rpy(self._end_effector_pose.pose.pose.orientation, degrees=True)
                    euler_target = quat2rpy(self._end_effector_target.pose.pose.orientation, degrees=True)

                    print(self.MSG_POSE.format(
                    self._end_effector_target.pose.pose.position.x,
                    self._end_effector_target.pose.pose.position.y,
                    self._end_effector_target.pose.pose.position.z,
                    euler_target[0], euler_target[1], euler_target[2],
                    self._end_effector_pose.pose.pose.position.x,
                    self._end_effector_pose.pose.pose.position.y,
                    self._end_effector_pose.pose.pose.position.z,
                    euler_current[0], euler_current[1], euler_current[2]))

        except Exception as e:
            print(e)

        finally:
            self._publish()

def main(args=None):
    if args is None:
        args = sys.argv

    execute(args=args)

if __name__ == '__main__':
    main()