import rospy
import smach
from geometry_msgs.msg import PoseStamped


class ReceiveMission(smach.StateMachine):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['ready'],
                             input_keys=['robot'],
                             output_keys=['goal_list'])
        self.missionReady = False
        self.mission = PoseStamped()

    def mission_callback(self, mission):
        self.mission = mission
        self.missionReady = True

    def execute(self, userdata):
        sub_mission = rospy.Subscriber(userdata.robot + '/mission', PoseStamped, self.mission_callback)

        while not self.missionReady:
            pass

        userdata.goal_list.append(self.mission)
        return 'mission_received'
