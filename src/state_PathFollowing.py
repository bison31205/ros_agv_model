import rospy
import smach
import tf2_ros
import tf2_geometry_msgs
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist


class PathFollowing(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['EOP','again'],
                             input_keys=['robot','path','speed'])
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_list = tf2_ros.TransformListener(self.tf_buffer)
        self.robot = ""

    def get_transform(self):
        while not rospy.is_shutdown():
            try:
                transform = self.tf_buffer.lookup_transform(self.robot + "/base_link", "world",
                                                            rospy.Time(0), rospy.Duration(1.0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException):
                rospy.logwarn("Failed to get transformation")
            else:
                return transform

    def compute_velocity_command(self, new_pose, lin_vel):
        transform = self.get_transform()

        pose_transformed = tf2_geometry_msgs.do_transform_pose(new_pose, transform)
        dist_x = pose_transformed.pose.position.x
        dist_y = pose_transformed.pose.position.y
        curvature = (2 * dist_y) / (dist_x ** 2 + dist_y ** 2)

        msg = Twist()
        msg.linear.x = math.copysign(lin_vel, dist_x)
        msg.angular.z = math.copysign(lin_vel, dist_x) * curvature
        if abs(msg.angular.z) >  0.3:
            math.copysign(0.3, msg.angular.z)
        return msg

    def find_next_pose(self, path):
        transform = self.get_transform()
        pose_transformed = tf2_geometry_msgs.do_transform_pose(path.poses[0], transform)
        dist_x = pose_transformed.pose.position.x
        dist_y = pose_transformed.pose.position.y
        min_dist = math.sqrt(dist_x ** 2 + dist_y ** 2)
        min_index = 0

        for pose, index in zip(path.poses[1:], range(1, len(path.poses))):
            pose_transformed = tf2_geometry_msgs.do_transform_pose(pose, transform)
            dist_x = pose_transformed.pose.position.x
            dist_y = pose_transformed.pose.position.y
            new_dist = math.sqrt(dist_x ** 2 + dist_y ** 2)
            if new_dist < min_dist:
                min_dist = new_dist
                min_index = index

        if min_index + 1 == len(path.poses):
            return "EOP" # END_OF_PATH
        else:
            return path.poses[min_index+1]

    def execute(self, userdata):
        self.robot = userdata.robot
        publisher = rospy.Publisher(self.robot + "/cmd_vel", Twist, queue_size=10)
        next_pose = self.find_next_pose(userdata.path)
        print next_pose
        if next_pose == "EOP":
            rospy.logwarn("EOP")
            return "EOP"
        else:
            goal_reached = False
            while not goal_reached:
                publisher.publish(self.compute_velocity_command(next_pose, userdata.speed))
                transform = self.tf_buffer.lookup_transform(self.robot + "/base_link", "world",
                                                            rospy.Time(0), rospy.Duration(1.0))
                pose_transformed = tf2_geometry_msgs.do_transform_pose(next_pose, transform)
                dist_x = pose_transformed.pose.position.x
                dist_y = pose_transformed.pose.position.y
                if math.sqrt(dist_x**2 + dist_y**2) < 0.1:
                    goal_reached = True
            # tell robot to stop
            publisher.publish(Twist())
            return "again"
