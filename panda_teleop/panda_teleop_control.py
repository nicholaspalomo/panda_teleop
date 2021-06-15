# Copyright (C) 2021 Bosch LLC CR, North America. All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

# TODO: Have a model that spawns which does not have any collision meshses. For now, just try to get it working with basic teleop
# TODO: This class only subscribes to joint states and publishes to end_effector_target
# TODO: Implement a software stop to set the end_effector_target to the current location if a problem is encountered
# TODO: Make sure that the position and rotation limits are enforced

import numpy as np
from numpy import random
from scipy.spatial.transform import Rotation as R
import copy
from typing import List

# ROS2 Python API libraries
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.client import Client

# ROS2 message and service data structures
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from rcl_interfaces.srv import GetParameters
from std_srvs.srv import Empty

# For teleop control
import curses
from rclpy.duration import Duration
from rclpy.qos import qos_profile_system_default
from std_msgs.msg import Header

# For 'q' keystroke exit
import os
import signal
import time

# Configure numpy output
np.set_printoptions(precision=4, suppress=True)

def execute(stdsrc, args):
    rclpy.init(args=args)

    app = PandaTeleop(TextWindow(stdsrc))
    app.poll_keys()

    app.destroy_node()
    rclpy.shutdown()

def quat2rpy(quat: Quaternion, degrees=True) -> List[float]:

    if degrees:
        return [angle * 180./np.pi for angle in list(R.from_quat([quat.x, quat.y, quat.z, quat.w]).as_euler())] # return the roll-pitch-yaw in degrees...

    else:
        return list(R.from_quat([quat.x, quat.y, quat.z, quat.w]).as_euler()) # ...or in radians

def rpy2quat(rpy: List[float], input_in_degrees=False) -> Quaternion:

    quat = R.from_euler('xyz', rpy, degrees=input_in_degrees).as_quat()

    out = Quaternion
    out.x = quat[0]
    out.y = quat[1]
    out.z = quat[2]
    out.w = quat[3]

    return out # return roll-pith-yaw angles as quaternion

class TextWindow():

    _screen = None
    _window = None
    _num_lines = None

    def __init__(self, stdscr, lines=25):
        self._screen = stdscr
        self._screen.nodelay(True)
        curses.curs_set(0)

        self._num_lines = lines

    def read_key(self):
        keycode = self._screen.getch()
        return keycode if keycode != -1 else None

    def clear(self):
        self._screen.clear()

    def write_line(self, lineno, message):
        if lineno < 0 or lineno >= self._num_lines:
            raise ValueError('lineno out of bounds')
        height, width = self._screen.getmaxyx()
        y = (height / self._num_lines) * lineno
        x = 10
        for text in message.split('\n'):
            text = text.ljust(width)
            self._screen.addstr(int(y), int(x), text)
            y += 1

    def refresh(self):
        self._screen.refresh()

    def beep(self):
        curses.flash()

class PandaTeleop(Node):
    def __init__(self, interface):
        super().__init__('panda_teleop_control')

        self._interface = interface
        self._running = True
        self._last_pressed = {}

        self.declare_parameters(
            namespace='',
            parameters=[
                ('base_frame', None),
                ('end_effector_frame', None),
            ]
        )
        self._hz = self.declare_parameter('hz', 10.0).value

        # Create end effector target publisher
        self._end_effector_target_publisher: Publisher = self.create_publisher(Odometry, self.get_parameter('end_effector_target_topic').value, 10)
        self._end_effector_pose_subscriber: Subscription = self.create_subscription(Odometry, self.get_parameter('end_effector_pose_topic').value, self.callback_end_effector_odom, 10)

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
        self._end_effector_target_origin.header.frame_id = self.get_parameter('base_frame').value
        self._end_effector_target_origin.header.child_frame_id = self.get_parameter('end_effector_frame').value
        self._end_effector_target_origin.header.stamp = self.get_clock().now().to_msg()

        self._end_effector_target: Odometry = copy.deepcopy(self._end_effector_target_origin)
        self._end_effector_pose: Odometry = copy.deepcopy(self._end_effector_target)

        # publish the initial end effector target, which corresponds to the joints at their neutral position
        self._end_effector_target_publisher.publish(self._end_effector_target)

        self.MSG_TELEOP = """
Reading from the keyboard and publishing to end effector target!
---------------------------
Linear movement:
u    i    o               +x    
j    k    l   -->   +y          -y
m    ,    .               -x    
For rotation, hold down the shift key:
---------------------------
U    I    O               +p    
J    K    L   -->   -r          +r
M    <    >               -p    
t : up (+z)
b : down (-z)
y : +yaw
n : -yaw
z : open grippers/close grippers
q : stop (quit)
CTRL-C to quit
        """
        self.MSG_POSE = """CURRENT END EFFECTOR TARGET POSE:
[x, y, z] = [{}, {}, {}] m
[r, p, y] = [{}, {}, {}] rad (Euler)
CURRENT END EFFECTOR POSE:
[x, y, z] = [{}, {}, {}] m
[r, p, y] = [{}, {}, {}] rad (Euler)"""

        self._planar_translation_bindings = { # +x, -x, +y, -y
            'u' : (1, 0, 1, 0),
            'i' : (1, 0, 0, 0),
            'o' : (1, 0, 1, -1),
            'j' : (0, 0, 1, 0),
            'l' : (0, 0, 0, -1),
            'm' : (0, -1, 1, 0),
            ',' : (0, -1, 0, 0),
            '.' : (0, -1, 0, -1)
        }

        self._vertical_translation_bindings = { # +z, -z
            't' : (1, 0),
            'b' : (0, -1)
        }

        self._planar_rotation_bindings = { # +p -p +r -r
            'U' : (1, 0, 0, -1),
            'I' : (1, 0, 0, 0),
            'O' : (1, 0, 1, 0),
            'J' : (0, 0, 0, -1),
            'L' : (0, 0, 1, 0),
            'M' : (0, -1, 0, -1),
            '<' : (0, -1, 0, 0),
            '>' : (0, -1, 1, 0)
        }

        self._yaw_rotation_bindings = { # +yaw, -yaw
            'y' : (1, 0),
            'n' : (0, -1)
        }

        self._open_close_gripper_bindings = {
            'z' : 0
        }

        self._translation_limits = [[-0.4, 0.4], [-0.4, 0.4], [0.0, 0.5]] # xyz
        self._rotation_limits = [[-45., 45.], [-45., 45.], [-45., 45.]] # rpy
        self._dtheta = 1.0
        self._dx = 0.01

    def callback_end_effector_odom(self, odom: Odometry):
        self._end_effector_pose = odom

    def _key_pressed(self, keycode):
        if keycode == ord('q'):
            self._running = False
            os.kill(os.getpid(), signal.SIGINT)

        elif keycode in self._planar_translation_bindings.keys():
            self._last_pressed[keycode] = self.get_clock().now()

        elif keycode in self._vertical_translation_bindings.keys():
            self._last_pressed[keycode] = self.get_clock().now()

        elif keycode in self._planar_rotation_bindings.keys():
            self._last_pressed[keycode] = self.get_clock().now()

        elif keycode in self._yaw_rotation_bindings.keys():
            self._last_pressed[keycode] = self.get_clock().now()

        elif keycode in self._open_close_gripper_bindings.keys():

            # Call the service to actuate the gripper
            future = self._actuate_gripper_client.call_async(Empty())
            if future.done():
                try:
                    response = future.result()
                except Exception as e:
                    self.get_logger().info('SERVICE CALL TO ACTUATE GRIPPER SERVICE FAILED %r' % (e,))
                else:
                    self.get_logger().info('GRIPPER ACTUATED SUCCESSFULLY')
        else:
            return

    def _set_pose_target(self):
        now = self.get_clock().now()
        keys = []
        for a in self._last_pressed:
            if now - self._last_pressed[a] < Duration(seconds=0.4):
                keys.append(a)

        for k in keys:
            if k in self._planar_translation_bindings.keys():
                binding = self._planar_translation_bindings[k]
                
                for i in range(2):
                    self._end_effector_target.pose.pose.position.x = np.clip(self._end_effector_target.pose.pose.position.x + binding[i] * self._dx, self._translation_limits[0][0], self._translation_limits[0][1])

                    self._end_effector_target.pose.pose.position.y = np.clip(self._end_effector_target.pose.pose.position.y + binding[i+2] * self._dx, self._translation_limits[1][0], self._translation_limits[1][1])

            elif k in self._vertical_translation_bindings.keys():
                binding = self._vertical_translation_bindings[k]

                self._end_effector_target.pose.pose.position.z = np.clip(self._end_effector_target.pose.pose.position.z + binding[0] * self._dx, self._translation_limits[2][0], self._translation_limits[2][1])

                self._end_effector_target.pose.pose.position.z = np.clip(self._end_effector_target.pose.pose.position.z + binding[1] * self._dx, self._translation_limits[2][0], self._translation_limits[2][1])

            elif k in self._planar_rotation_bindings.keys():
                binding = self._planar_rotation_bindings[k]

                # Get the current target rpy (Euler angles) for the end effector
                euler_target = quat2rpy(self._end_effector_target.pose.pose.orientation, degrees=True)

                for i in range(2):
                    euler_target[0] = np.clip(euler_target[0] + binding[i+2] * self._dtheta, self._rotation_limits[0][0], self._rotation_limits[0][1])

                    euler_target[1] = np.clip(euler_target[1] + binding[i] * self._dtheta, self._rotation_limits[1][0], self._rotation_limits[1][1])

                # Convert rotation back to radians and back to a quaternion representation
                quat = rpy2quat(euler_target, input_in_degrees=True)
                self._end_effector_target.pose.pose.orientation = copy.deepcopy(quat)

            elif k in self._yaw_rotation_bindings.keys(): # yaw rotation
                binding = self._yaw_rotation_bindings[k]

                euler_target = quat2rpy(self._end_effector_target.pose.pose.orientation, degrees=True)

                euler_target[2] = np.clip(euler_target[2] + binding[0] * self._dtheta, self._translation_limits[2][0], self._translation_limits[2][1])

                euler_target[2] = np.clip(euler_target[2] + binding[1] * self._dtheta, self._rotation_limits[2][0], self._rotation_limits[2][1])

                # Convert rotation back to radians and back to a quaternion representation
                quat = rpy2quat(euler_target, input_in_degrees=True)
                self._end_effector_target.pose.pose.orientation = copy.deepcopy(quat)

            else:
                return

    def _publish(self):

        euler_current = quat2rpy(self._end_effector_pose.pose.pose.orientation, degrees=True)
        euler_target = quat2rpy(self._end_effector_target.pose.pose.orientation, degrees=True)

        self._interface.clear()
        self._interface.write_line(1, self.MSG_TELEOP)
        self._interface.write_line(18, self.MSG_POSE.format(
        self._end_effector_target.pose.pose.position.x,
        self._end_effector_target.pose.pose.position.y,
        self._end_effector_target.pose.pose.position.z,
        euler_target[0], euler_target[1], euler_target[2],
        self._end_effector_pose.pose.pose.position.x,
        self._end_effector_pose.pose.pose.position.y,
        self._end_effector_pose.pose.pose.position.z,
        euler_current[0], euler_current[1], euler_current[2]))
        self._interface.refresh()

        self._end_effector_target_publisher.publish(self._end_effector_target)

    def poll_keys(self):
        self._running = True
        while self._running:
            while True:
                keycode = self._interface.read_key()
                if keycode is None:
                    break
                self._key_pressed(keycode)
                self._set_pose_target()
                self._publish()
                time.sleep(1.0/self._hz)

def main(args=None):
    try:
        curses.wrapper(execute, args=args)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()