import rospy
import smach


class ProcessMap(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['map_segmented'],
                             input_keys=['map'],
                             output_keys=['zones'])

    def execute(self, userdata):
        return 'map_segmented'
