import rospy
import smach
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose


class Start(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['init_complete'],
                             input_keys=['robot'],
                             output_keys=['map', 'pose'])
        self.mapReady = False
        self.poseReady = False
        self.map = OccupancyGrid()
        self.pose = Pose()

    def map_callback(self, map, userdata):
        userdata.map = map
        self.mapReady = True
        rospy.loginfo("{"+userdata.robot+"} Map received")

    def pose_callback(self, pose, userdata):
        userdata.pose = map
        self.poseReady = True
        rospy.loginfo("{"+userdata.robot+"} Initial pose received")

    def execute(self, userdata):
        sub_map = rospy.Subscriber(userdata.robot + "/map", OccupancyGrid, self.map_callback, userdata)
        sub_pose = rospy.Subscriber(userdata.robot + "/initialpose", Pose, self.pose_callback, userdata)

        rospy.loginfo("{"+userdata.robot+"} Waiting for map and inital pose")

        while not (self.mapReady and self.poseReady):
            pass

        userdata.map = self.map
        userdata.pose = self.pose
        return 'init_complete'
