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
from state_GetPath import GetPath


class RobotModel():
    def __init__(self):
        rospy.init_node("robot_node")
        self.robot = rospy.get_param('name')
        
        print self.robot
        
        # Create a SMACH state machine
        self.sm = smach.StateMachine(outcomes=['END'])
        self.sm.userdata.robot = self.robot
        self.sm.userdata.speed = 0.15
        self.sm.userdata.path = Path()

        # Open the container
        with self.sm:
            # Add states to the container
            smach.StateMachine.add('GetPath', GetPath(),
                                   transitions={'follow_path':'PathFollowing'})
            smach.StateMachine.add('PathFollowing', PathFollowing(),
                                   transitions={'again':'PathFollowing',
                                                'EOP':'GetPath'})

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
    asd = RobotModel()
    asd.start()
