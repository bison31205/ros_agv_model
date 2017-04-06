import rospy
import smach
import tf2_ros
import tf2_geometry_msgs
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class GetPath(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['follow_path'],
                             input_keys=['robot','goal'],
                             output_keys=['path'])
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_list = tf2_ros.TransformListener(self.tf_buffer)
        self.robot = ""
        self.path_received = False
        self.path = Path()
        
    def execute(self, userdata):
        self.robot = userdata.robot      
        #publisher = rospy.Publisher(self.robot + "/goal", PoseStamped, queue_size=10)
        subscriber = rospy.Subscriber(self.robot + "/plan", Path, self.planCallback)
        #publisher.publish(userdata.goal)
        
        while not self.path_received: pass
        userdata.path = self.path
        return 'follow_path'
        
        
    def planCallback(self, path):
        self.path = path
        self.path_received = True
