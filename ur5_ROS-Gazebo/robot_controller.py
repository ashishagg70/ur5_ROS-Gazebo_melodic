#!/usr/bin/env python

"""
    moveit_cartesian_path.py - Version 0.1 2016-07-28

    Based on the R. Patrick Goebel's moveit_cartesian_demo.py demo code.

    Plan and execute a Cartesian path for the end-effector.

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2014 Patrick Goebel.  All rights reserved.
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:

    http://www.gnu.org/licenses/gpl.html
"""

import rospy, sys, numpy as np
import moveit_commander
from copy import deepcopy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point, Pose, Quaternion
import moveit_msgs.msg
import geometry_msgs
from sensor_msgs.msg import Image
from ur5_notebook.msg import Tracker
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from ur5_notebook.msg import blocks_info
import tf
from time import sleep
tracker = Tracker()



class robot_controller:
    def blocks_info_callback(self, msg):
        rospy.loginfo("ashish2: "+msg.name)
    def __init__(self):
        rospy.init_node("robot_controller", anonymous=False)

        rospy.loginfo("Starting node robot_controller")

        rospy.on_shutdown(self.cleanup)
        self.block_info_sub = rospy.Subscriber('blocks_info', blocks_info, callback= self.blocks_info_callback)

    def cleanup(self):
        rospy.loginfo("Stopping the robot_controller")

mp=robot_controller()

rospy.spin()
