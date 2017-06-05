import rospy
import smach
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class GetPath(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['path_received'],
                             input_keys=['robot', 'goal_list'],
                             output_keys=['path', 'segment_index', 'speed', 'trajectory'])
        self.pathReady = False
        self.path = Path()

    def plan_callback(self, path):
        self.path = path
        self.pathReady = True
        
    def execute(self, userdata):
        pub_goal = rospy.Publisher(userdata.robot + "/goal", PoseStamped, queue_size=10, latch=True)
        pub_goal.publish(userdata.goal_list[0])

        rospy.sleep(1)

        sub_plan = rospy.Subscriber(userdata.robot + "/plan", Path, self.plan_callback)
        rospy.loginfo("{" + userdata.robot + "} Waiting for path to goal")
        while not self.pathReady:
            pass

        rospy.loginfo("{" + userdata.robot + "} Path received")

        userdata.path = self.path
        userdata.segment_index = 0
        userdata.speed = []
        userdata.trajectory = []
        return 'path_received'
