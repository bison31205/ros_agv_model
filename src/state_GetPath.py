import rospy
import smach


class GetPath(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['path_received'],
                             input_keys=['robot', 'goal_list',
                                         'path_ready_event', 'path',
                                         'pub_goal'],
                             output_keys=['speed',
                                          'trajectory', 'path_ready_event',
                                          'pub_goal', 'segment_length'])

    def execute(self, userdata):
        print userdata.goal_list[0].header.seq
        userdata.pub_goal.publish(userdata.goal_list[0])

        rospy.loginfo("{" + userdata.robot + "} Waiting for path to goal")
        userdata.path_ready_event.wait()
        userdata.path_ready_event.clear()
        rospy.loginfo("{" + userdata.robot + "} Path received")

        userdata.speed = []
        userdata.trajectory = []
        userdata.segment_length = []
        return 'path_received'
