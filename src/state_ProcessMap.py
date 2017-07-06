import rospy
import smach
import yaml


class ProcessMap(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['map_segmented'],
                             input_keys=['map'],
                             output_keys=['map_zones'])

    def execute(self, userdata):
        temp_file = open(rospy.get_param('map_zones_file'))
        userdata.map_zones = yaml.load(temp_file)

        return 'map_segmented'
