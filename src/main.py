#!/usr/bin/env python

import rospy
import smach
import yaml

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from msg_pkg.msg import Features


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
from state_ExitState import ExitState


class RobotModel:
    def __init__(self):
        rospy.init_node("robot_node", log_level=rospy.INFO)
        self.robot = rospy.get_param('name')
        self.robot_list = rospy.get_param('robot_list')

        self.sub_robot_traj = dict()
        self.sub_robot_feat = dict()

        # Create a SMACH state machine
        self.sm = smach.StateMachine(outcomes=['END'])
        self.sm.userdata.robot = self.robot
        self.sm.userdata.robot_list = self.robot_list
        self.sm.userdata.robot_data = []
        self.sm.userdata.segment_time = 2.5  # seconds
        self.sm.userdata.max_speed = 0.15  # m/s
        self.sm.userdata.goal_list = []
        self.sm.userdata.goal_counter = [0, 0]
        self.sm.userdata.odom = Odometry()
        self.sm.userdata.path_ready = False
        self.sm.userdata.robots_trajectories = dict()
        self.sm.userdata.robots_features = dict()

        # Load robot model parameters
        self.param_file = rospy.get_param('parameters_file')
        self.load_learning_data()

        # Create SM subscribers
        self.odom_sub = rospy.Subscriber(self.robot + '/odom', Odometry, self.odom_callback, queue_size=1)
        self.goal_sub = rospy.Subscriber(self.robot + '/mission', PoseStamped, self.mission_callback, queue_size=1)
        self.path_sub = rospy.Subscriber(self.robot + '/plan', Path, self.plan_callback, queue_size=1)
        for robot in self.robot_list:
            self.sm.userdata.robots_trajectories[robot] = Path()
            self.sm.userdata.robots_features[robot] = Features()
            self.sub_robot_traj[robot] = rospy.Subscriber("trajectories/" + robot,
                                                          Path, self.trajectories_callback, robot, queue_size=1)
            self.sub_robot_feat[robot] = rospy.Subscriber("features/" + robot,
                                                          Features, self.features_callback, robot, queue_size=1)

        # Create SM publishers
        self.sm.userdata.pub_goal = rospy.Publisher(self.robot + "/goal", PoseStamped, queue_size=10, latch=True)
        self.sm.userdata.pub_speed = rospy.Publisher(self.robot + "/cmd_vel", Twist, queue_size=10, latch=True)
        self.sm.userdata.pub_path = rospy.Publisher(self.robot + "/follow_path", Path, queue_size=10, latch=True)
        self.sm.userdata.pub_trajectory = rospy.Publisher("trajectories/" + self.robot, Path, queue_size=10, latch=True)
        self.sm.userdata.pub_features = rospy.Publisher("features/" + self.robot, Features, queue_size=10, latch=True)

        # Open the container
        with self.sm:
            # Add states to the container
            smach.StateMachine.add('Start', Start(),
                                   transitions={'init_complete': 'ProcessMap'})
            smach.StateMachine.add('ProcessMap', ProcessMap(),
                                   transitions={'map_segmented': 'GoalAssignment'})
            smach.StateMachine.add('GoalAssignment', GoalAssignment(),
                                   transitions={'next_goal': 'GetPath',
                                                'no_new_goals': 'ExitState'})
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
                                                'change_speed': 'ChangeSpeed',
                                                'NaN': 'ExitState'})
            smach.StateMachine.add('AddIntermediateGoal', AddIntermediateGoal(),
                                   transitions={'goal_added': 'GoalAssignment'})
            smach.StateMachine.add('ChangeSpeed', ChangeSpeed(),
                                   transitions={'speed_changed': 'CalcTrajectory'})
            smach.StateMachine.add('ExitState', ExitState(),
                                   transitions={'END': 'END'})

    def odom_callback(self, data):
        self.sm.userdata.odom = data

    def mission_callback(self, data):
        self.sm.userdata.goal_list.append(data)
        self.sm.userdata.goal_counter[0] += 1

    def plan_callback(self, path):
        self.sm.userdata.path = path
        self.sm.userdata.path_ready = True

    def trajectories_callback(self, data, robot):
        self.sm.userdata.robots_trajectories[robot] = data

    def features_callback(self, data, robot):
        self.sm.userdata.robots_features[robot] = data

    def start(self):
        self.sm.execute()

    def load_learning_data(self):
        learning_data_file = open(self.param_file)
        self.sm.userdata.robot_data = yaml.load(learning_data_file)


if __name__ == '__main__':
    asd = RobotModel()
    asd.start()
