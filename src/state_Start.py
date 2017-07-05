import rospy
import smach
import threading
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


class Start(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['init_complete'],
                             input_keys=['robot', 'pub_trajectory'],
                             output_keys=['map', 'pose', 'pub_trajectory'])
        self.mapReady_event = threading.Event()
        self.poseReady_event = threading.Event()
        self.map = OccupancyGrid()
        self.pose = Pose()

    def map_callback(self, map, userdata):
        userdata.map = map
        self.mapReady_event.set()
        rospy.loginfo("{"+userdata.robot+"} Map received")

    def pose_callback(self, pose, userdata):
        userdata.pose = pose
        self.poseReady_event.set()
        rospy.loginfo("{"+userdata.robot+"} Initial pose received")

        # Publish dummy trajectory for initial position
        temp_traj = Path()
        temp_traj.header.frame_id = 'world'
        temp_pose = PoseStamped()
        temp_pose.pose = pose
        temp_traj.poses.append(temp_pose)

        userdata.pub_trajectory.publish(temp_traj)

    def execute(self, userdata):
        sub_map = rospy.Subscriber(userdata.robot + "/map", OccupancyGrid, self.map_callback, userdata)
        sub_pose = rospy.Subscriber(userdata.robot + "/initialpose", Pose, self.pose_callback, userdata)

        rospy.loginfo("{"+userdata.robot+"} Waiting for map and inital pose")

        self.mapReady_event.wait()
        self.poseReady_event.wait()

        userdata.map = self.map
        userdata.pose = self.pose
        return 'init_complete'
