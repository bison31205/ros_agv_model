#!/usr/bin/env python

import rospy
import smach

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


from state_Start import Start
from state_ProcessMap import ProcessMap
from state_GetPath import GetPath
from state_CalcTrajectory import CalcTrajectory
from state_CheckClearance import CheckClearance
from state_Driving import Driving
from state_ConflictResolver import ConflictResolver
from state_AddIntermediateGoal import AddIntermediateGoal
from state_ChangeSpeed import ChangeSpeed
from state_GoalAssignment import GoalAssignment


class RobotModel():
    def __init__(self):
        rospy.init_node("robot_node", log_level=rospy.INFO)
        self.robot = rospy.get_param('name')

        # Create a SMACH state machine
        self.sm = smach.StateMachine(outcomes=['END'])
        self.sm.userdata.robot = self.robot
        self.sm.userdata.segment_time = 5  # seconds
        self.sm.userdata.max_speed = 0.15  # m/s
        self.sm.userdata.goal_list = []
        self.sm.userdata.goal_counter = [0, 0]
        self.sm.userdata.odom = Odometry()

        # Create SM subscribers
        self.odom_sub = rospy.Subscriber(self.robot + '/odom', Odometry, self.odom_callback, queue_size=1)
        self.goal_sub = rospy.Subscriber(self.robot + '/mission', PoseStamped, self.mission_callback, queue_size=1)

        # Open the container
        with self.sm:
            # Add states to the container
            smach.StateMachine.add('Start', Start(),
                                   transitions={'init_complete': 'ProcessMap'})
            smach.StateMachine.add('ProcessMap', ProcessMap(),
                                   transitions={'map_segmented': 'GoalAssignment'})
            smach.StateMachine.add('GoalAssignment', GoalAssignment(),
                                   transitions={'next_goal': 'GetPath'})
            smach.StateMachine.add('GetPath', GetPath(),
                                   transitions={'path_received': 'CalcTrajectory'})
            smach.StateMachine.add('CalcTrajectory', CalcTrajectory(),
                                   transitions={'trajectory_created': 'CheckClearance'})
            smach.StateMachine.add('CheckClearance', CheckClearance(),
                                   transitions={'conflict': 'ConflictResolver',
                                                'OK_clearance': 'Driving'})
            smach.StateMachine.add('Driving', Driving(),
                                   transitions={'goal_reached': 'GoalAssignment',
                                                'next_segment': 'CheckClearance'})
            smach.StateMachine.add('ConflictResolver', ConflictResolver(),
                                   transitions={'just_drive': 'Driving',
                                                'change_path': 'AddIntermediateGoal',
                                                'change_speed': 'ChangeSpeed'})
            smach.StateMachine.add('AddIntermediateGoal', AddIntermediateGoal(),
                                   transitions={'goal_added': 'GoalAssignment'})
            smach.StateMachine.add('ChangeSpeed', ChangeSpeed(),
                                   transitions={'speed_changed': 'CalcTrajectory'})

    def odom_callback(self, data):
        self.sm.userdata.odom = data

    def mission_callback(self, data):
        self.sm.userdata.goal_list.append(data)
        self.sm.userdata.goal_counter[0] += 1

    def start(self):
        self.sm.execute()

if __name__ == '__main__':
    asd = RobotModel()
    asd.start()
