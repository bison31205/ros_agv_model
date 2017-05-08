#!/usr/bin/env python

import rospy
import smach


from state_Start import Start
from state_ProcessMap import ProcessMap
from state_ReceiveMission import ReceiveMission
from state_GetPath import GetPath
from state_CalcTrajectory import CalcTrajectory
from state_CheckClearance import CheckClearance
from state_Driving import Driving
from state_ConflictResolver import ConflictResolver
from state_AddIntermediateGoal import AddIntermediateGoal
from state_ChangeSpeed import ChangeSpeed
from state_GoalAssigment import GoalAssigment


class RobotModel():
    def __init__(self):
        rospy.init_node("robot_node")
        self.robot = rospy.get_param('name')
        
        print self.robot
        
        # Create a SMACH state machine
        self.sm = smach.StateMachine(outcomes=['END'])
        self.sm.userdata.robot = self.robot
        self.sm.userdata.sampling_time = 5  # seconds
        self.sm.userdata.max_speed = 0.15  # m/s

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
            smach.StateMachine.add('CalcTrajectory', CalcTrajectory(),
                                   transitions={'trajectory_created': 'CheckClearance'})
            smach.StateMachine.add('CheckClearance', CheckClearance(),
                                   transitions={'conflict': 'ConflictResolver',
                                                'OK_clearance': 'Driving'})
            smach.StateMachine.add('Driving', Driving(),
                                   transitions={'goal_reached': 'GoalAssigment',
                                                'next_segment': 'CheckClearance'})
            smach.StateMachine.add('GoalAssigment', GoalAssigment(),
                                   transitions={'next_goal': 'CalcTrajectory',
                                                'new_mission' : 'ReceiveMission'})

            smach.StateMachine.add('ConflictResolver', ConflictResolver(),
                                   transitions={'just_drive': 'Driving',
                                                'change_path': 'AddIntermediateGoal',
                                                'change_speed': 'ChangeSpeed'})
            smach.StateMachine.add('AddIntermediateGoal', AddIntermediateGoal(),
                                   transitions={'goal_added': 'GetPath'})
            smach.StateMachine.add('ChangeSpeed', ChangeSpeed(),
                                   transitions={'speed_changed': 'CalcTrajectory'})

    def start(self):
        self.sm.execute()

if __name__ == '__main__':
    asd = RobotModel()
    asd.start()
