import rospy
import smach
import yaml


class ProcessMap(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['map_segmented'],
                             input_keys=['map'],
                             output_keys=['zones', 'map_segments'])

    def execute(self, userdata):
        temp_file = open(rospy.get_param('map_segments_file'))
        userdata.map_segments = yaml.load(temp_file)

        return 'map_segmented'
