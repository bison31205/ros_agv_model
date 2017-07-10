#!/usr/bin/env python

import rospy
import smach
import yaml
import threading

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from msg_pkg.msg import Features
from msg_pkg.msg import Statistics


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
        self.sm.userdata.segment_time = 7  # seconds
        self.sm.userdata.max_speed = 0.2  # m/s
        self.sm.userdata.goal_list = []
        self.sm.userdata.goal_counter = [0, 0, []]
        self.sm.userdata.odom = Odometry()
        self.sm.userdata.robots_trajectories = dict()
        self.sm.userdata.robots_features = dict()

        self.sm.userdata.trajectory_updated_robots = []
        self.sm.userdata.features_updated_robots = []
        self.sm.userdata.ignore_conflict_robots = []
        self.sm.userdata.trajectory_updated_event = threading.Event()
        self.sm.userdata.path_ready_event = threading.Event()
        self.sm.userdata.new_odom_event = threading.Event()
        self.sm.userdata.new_mission_event = threading.Event()
        self.sm.userdata.features_updated_event = threading.Event()

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
        self.sm.userdata.pub_features = rospy.Publisher("features/" + self.robot, Features, queue_size=20, latch=True)
        self.sm.userdata.pub_exit = rospy.Publisher("exit_state/" + self.robot, Statistics, queue_size=10, latch=True)

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
                                   transitions={'continue': 'CheckClearance',
                                                'change_path': 'AddIntermediateGoal',
                                                'change_speed': 'ChangeSpeed',
                                                'recalculate_trajectory': 'CalcTrajectory',
                                                'NaN': 'ExitState'})
            smach.StateMachine.add('AddIntermediateGoal', AddIntermediateGoal(),
                                   transitions={'goal_added': 'GoalAssignment'})
            smach.StateMachine.add('ChangeSpeed', ChangeSpeed(),
                                   transitions={'speed_changed': 'CalcTrajectory'})
            smach.StateMachine.add('ExitState', ExitState(),
                                   transitions={'END': 'END'})

    def odom_callback(self, data):
        self.sm.userdata.odom = data
        self.sm.userdata.new_odom_event.set()

    def mission_callback(self, data):
        self.sm.userdata.goal_list.append(data)
        self.sm.userdata.goal_counter[0] += 1
        self.sm.userdata.new_mission_event.set()

    def plan_callback(self, path):
        self.sm.userdata.path = path
        self.sm.userdata.path_ready_event.set()

    def trajectories_callback(self, data, robot):
        raise_event = False
        # Check if new trajectory is subset of the old one (without driven segments)
        if not all(pose in self.sm.userdata.robots_trajectories[robot].poses
                   for pose in data.poses):
            raise_event = True

        self.sm.userdata.robots_trajectories[robot] = data

        if raise_event:
            if robot in self.sm.userdata.ignore_conflict_robots:
                self.sm.userdata.ignore_conflict_robots.remove(robot)
            self.sm.userdata.trajectory_updated_robots.append(robot)
            self.sm.userdata.trajectory_updated_event.set()

    def features_callback(self, data, robot):
        self.sm.userdata.robots_features[robot] = data
        self.sm.userdata.features_updated_robots.append(robot)
        self.sm.userdata.features_updated_event.set()

    def start(self):
        self.sm.execute()

    def load_learning_data(self):
        learning_data_file = open(self.param_file)
        self.sm.userdata.robot_data = yaml.load(learning_data_file)


if __name__ == '__main__':
    asd = RobotModel()
    asd.start()
