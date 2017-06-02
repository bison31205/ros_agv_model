import rospy
import smach


class ChangeSpeed(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['speed_changed'],
                             input_keys=['map'],
                             output_keys=['zones'])

    def execute(self, userdata):
        return 'speed_changed'

