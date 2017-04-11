import rospy
import smach


class Start(smach.StateMachine):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['ready'],
                             input_keys=['map'],
                             output_keys=['zones'])

    def execute(self, userdata):
        return 'map_segmented'

