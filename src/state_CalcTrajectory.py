import rospy
import smach
from nav_msgs.msg import Path

class CalcTrajectory(smach.StateMachine):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['trajectory_created'],
                             input_keys=['path', 'speed', 'max_speed', 'sampling_time'],
                             output_keys=['trajectory'])

    def execute(self, userdata):
        spent_time = 0
        userdata.trajectory = Path()
        pose_old = userdata.path.poses[0]
        last_time = rospy.get_time()

        for pose in userdata.path.poses[1:]:
            if len(userdata.trajectory) in userdata.speed:
                speed = userdata.speed[len(userdata.trajectory)]
            else
                speed = userdata.max_speed

            dist = math.sqrt((pose_old.position.x - pose.position.x) ** 2 +
                             (pose_old.position.y - pose.position.y) ** 2)

            next_pose_time = dist / speed
            if spent_time + next_pose_time > userdata.sampling_time or pose == userdata.path.poses[-1]:
                last_time += spent_time + next_pose_time
                userdata.trajectory.poses.append(pose)
                userdata.trajectory.poses[-1].header.seq = len(userdata.trajectory.poses) - 1
                userdata.trajectory.poses[-1].header.time = rospy.Time.from_sec(last_time)
                spent_time = 0

            pose_old = pose_new

        return 'trajectory_created'
