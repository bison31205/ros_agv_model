import rospy
import smach
import math
from nav_msgs.msg import Path


class CalcTrajectory(smach.StateMachine):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['trajectory_created'],
                             input_keys=['path', 'speed', 'max_speed', 'sampling_time', 'segment_index'],
                             output_keys=['trajectory'])

    def execute(self, userdata):
        spent_time = 0
        userdata.trajectory = [Path()]
        pose_old = userdata.path.poses[0]
        last_time = rospy.get_time()

# potrebno je implementirati da update-a trajektoriju samo od
# POSE_INDEX segmenta nadalje

        for pose in userdata.path.poses[1:]:
            if len(userdata.trajectory.poses) in userdata.speed:
                speed = userdata.speed[len(userdata.trajectory.poses)]
            else:
                speed = userdata.max_speed

            dist = math.sqrt((pose_old.position.x - pose.position.x) ** 2 +
                             (pose_old.position.y - pose.position.y) ** 2)

            next_pose_time = dist / speed

            last_time += next_pose_time
            userdata.trajectory[-1].poses.append(pose)
            userdata.trajectory[-1].poses[-1].header.seq = len(userdata.trajectory.poses) - 1
            userdata.trajectory[-1].poses[-1].header.time = rospy.Time.from_sec(last_time)

            if spent_time + next_pose_time > userdata.sampling_time or pose == userdata.path.poses[-1]:
                spent_time = 0
                userdata.trajectory.append(Path())
            else:
                spent_time += next_pose_time

            pose_old = pose

        return 'trajectory_created'
