#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
import tf2_ros
import tf2_geometry_msgs
import itertools
import math

from state_Start import Start
from state_ProcessMap import ProcessMap
from state_ReceiveMission import ReceiveMission
from state_GetPath import GetPath
# from state_CalcTrajectory import CalcTrajectory
# from state_CheckClearance import CheckClearance
# from state_PathFollowing import PathFollowing
# from state_ConflictResolver import ConflictResolver
# from state_AddIntermidiateGoal import AddIntermidiateGoal
# from state_ChangeSpeed import ChangeSpeed
# from state_GoalAssigment import GoalAssigment




class RobotModel():
    def __init__(self):
        rospy.init_node("robot_node")
        self.robot = rospy.get_param('name')
        
        print self.robot
        
        # Create a SMACH state machine
        self.sm = smach.StateMachine(outcomes=['END'])
        self.sm.userdata.robot = self.robot

        # Open the container
        with self.sm:
            # Add states to the container
            smach.StateMachine.add('Start', Start(),
                                   transitions={'init_complete': 'ProcessMap'})
            smach.StateMachine.add('ProcessMap', ProcessMap(),
                                   transitions={'map_segmented': 'ReceiveMission'})
            smach.StateMachine.add('ReceiveMission', ReceiveMission(),
                                   transitions={'mission_received': 'GetPath'})
            smach.StateMachine.add('GetPath', GetPath(),
                                   transitions={'path_received': 'CalcTrajectory'})


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
