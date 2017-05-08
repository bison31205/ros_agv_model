import rospy
import smach
from nav_msgs.msg import Path

sampling_time = 1 # seconds
max_speed = 0.15 # m/s

class CheckClearance(smach.StateMachine):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['conflict', 'OK_clearance'],
                             input_keys=['trajectory'],
                             output_keys=['trajectory','conflict_features'])

    def execute(self, userdata):

        return 'trajectory_created'