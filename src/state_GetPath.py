import rospy
import smach
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class GetPath(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['follow_path'],
                             input_keys=['robot', 'goal_list'],
                             output_keys=['path', 'speed'])
        self.pathReady = False
        self.path = Path()

    def plan_callback(self, path):
        self.path = path
        self.pathReady= True
        
    def execute(self, userdata):
        pub_goal = rospy.Publisher(userdata.robot + "/goal", PoseStamped, queue_size=10)
        sub_plan = rospy.Subscriber(userdata.robot + "/plan", Path, self.plan_callback)

        pub_goal.publish(userdata.goal_list[0])
        
        while not self.pathReady:
            pass

        userdata.path = self.path
        userdata.speed = dict()
        return 'path_received'
