import rospy
import smach
from geometry_msgs.msg import PoseStamped


class GetPath(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['path_received'],
                             input_keys=['robot', 'goal_list',
                                         'path_ready', 'path',
                                         'pub_goal'],
                             output_keys=['speed',
                                          'trajectory', 'path_ready',
                                          'pub_goal'])

    def execute(self, userdata):
        userdata.path_ready = False
        userdata.pub_goal.publish(userdata.goal_list[0])
        rospy.loginfo("{" + userdata.robot + "} Waiting for path to goal")
        while not userdata.path_ready:
            pass

        rospy.loginfo("{" + userdata.robot + "} Path received")

        userdata.speed = []
        userdata.trajectory = []
        return 'path_received'
