import rospy
import smach
from nav_msgs.msg import Path

sampling_time = 1 # seconds
max_speed = 0.15 # m/s

class GoalAssigment(smach.StateMachine):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['goal_reached', 'middle_of_path'],
                             input_keys=['trajectory', 'speed', 'max_speed'],
                             output_keys=['trajectory'])

    def execute(self, userdata):

        return 'trajectory_created'
