import rospy
import smach
import math
from nav_msgs.msg import Path


class CalcTrajectory(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['trajectory_created'],
                             input_keys=['path', 'speed', 'max_speed',
                                         'segment_time', 'trajectory'],
                             output_keys=['trajectory', 'speed'])

    @staticmethod
    def prepare_new_trajectory(path, max_speed, segment_time):
        spent_time = 0
        trajectory = []
        speed = []
        new_segment = Path()
        new_segment.header.frame_id = 'world'
        pose_old = path.poses[0]
        last_time = rospy.get_time()

        for pose in path.poses[1:]:
            dist = math.sqrt((pose_old.pose.position.x - pose.pose.position.x) ** 2 +
                             (pose_old.pose.position.y - pose.pose.position.y) ** 2)

            next_pose_time = dist / max_speed

            last_time += next_pose_time
            new_segment.poses.append(pose)
            new_segment.poses[-1].header.seq = len(new_segment.poses) - 1
            new_segment.poses[-1].header.stamp = rospy.Time.from_sec(last_time)

            if spent_time + next_pose_time > segment_time or pose == path.poses[-1]:
                trajectory.append(new_segment)
                speed.append(max_speed)

                new_segment = Path()
                new_segment.header.frame_id = 'world'
                spent_time = 0
            else:
                spent_time += next_pose_time

            pose_old = pose
        return [trajectory, speed]

    @staticmethod
    def recalculate_trajectory(trajectory, speed):
        last_time = rospy.get_time()
        pose_old = trajectory[0].poses[0]

        for segment, seg_speed in (trajectory, speed):
            for pose in segment.poses[1:]:
                dist = math.sqrt((pose_old.position.x - pose.position.x) ** 2 +
                                 (pose_old.position.y - pose.position.y) ** 2)

                next_pose_time = dist / seg_speed

                last_time += next_pose_time
                pose.stamp = rospy.Time.from_sec(last_time)

                pose_old = pose
        return trajectory

    def execute(self, userdata):
        rospy.loginfo(len(userdata.path.poses))
        if len(userdata.speed) == 0:
            [userdata.trajectory, userdata.speed] = self.prepare_new_trajectory(userdata.path,
                                                                                userdata.max_speed,
                                                                                userdata.segment_time)
        else:
            userdata.trajectory = self.recalculate_trajectory(userdata.trajectory,
                                                              userdata.speed)

        return 'trajectory_created'
