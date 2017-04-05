#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
import tf2_ros
import tf2_geometry_msgs
import itertools
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

from state_PathFollowing import PathFollowing


class RobotModel():
    def __init__(self, robot):
        self.robot = robot
        rospy.init_node(self.robot + "_node")

        # Create a SMACH state machine
        self.sm = smach.StateMachine(outcomes=['END'])
        self.sm.userdata.robot = self.robot
        self.sm.userdata.speed = 0.15

        pos1 = PoseStamped()
        pos1.pose.position.x = 1
        pos1.pose.position.y = 0
        pos2 = PoseStamped()
        pos2.pose.position.x = 3
        pos2.pose.position.y = 0
        pos3 = PoseStamped()
        pos3.pose.position.x = 3
        pos3.pose.position.y = 5
        pos4 = PoseStamped()
        pos4.pose.position.x = 1
        pos4.pose.position.y = 5
        pos5 = PoseStamped()
        pos5.pose.position.x = 1
        pos5.pose.position.y = 0
        path = Path()
        path.poses = [pos1, pos2, pos3, pos4, pos5]

        self.sm.userdata.path = path

        # Open the container
        with self.sm:
            # Add states to the container
            smach.StateMachine.add('PathFollowing', PathFollowing(),
                                   transitions={'again':'PathFollowing',
                                                'EOP':'END'})

    def start(self):
        self.sm.execute()

    def n_sphere(self, feat, weight, param):
        dist = 0
        for f,w,p in itertools.izip(feat[1:], weight[1:], param[1:]):
            dist += (f * w - p) ** 2
        dist = math.sqrt(dist)
        if dist <= param[0]:
            return weight[0] * dist / param[0]
        else : return 'NaN'

if __name__ == '__main__':
    asd = RobotModel("mirko")
    asd.start()
