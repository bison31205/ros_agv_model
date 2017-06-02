import rospy
import smach
from geometry_msgs.msg import PoseStamped


class ReceiveMission(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['mission_received'],
                             input_keys=['robot', 'goal_list'],
                             output_keys=['goal_list'])
        self.missionReady = False
        self.mission = PoseStamped()

    def mission_callback(self, mission):
        self.mission = mission
        self.missionReady = True

    def execute(self, userdata):
        sub_mission = rospy.Subscriber(userdata.robot + '/mission', PoseStamped, self.mission_callback)

        rospy.loginfo("{"+userdata.robot+"} Waiting for mission")

        while not self.missionReady:
            pass

        rospy.loginfo("{"+userdata.robot+"} Mission received")

        userdata.goal_list.append(self.mission)
        return 'mission_received'
