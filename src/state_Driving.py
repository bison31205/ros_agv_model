import rospy, math
import smach
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist


class Driving(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['goal_reached', 'next_segment'],
                             input_keys=['robot', 'trajectory', 'odom',
                                         'speed', 'max_speed', 'new_odom_event',
                                         'goal_counter', 'goal_list', 'goal_time',
                                         'pub_speed', 'pub_path', 'pub_trajectory'],
                             output_keys=['trajectory', 'speed', 'new_odom_event',
                                          'goal_counter', 'goal_list',
                                          'pub_speed', 'pub_path', 'pub_trajectory'])

    @staticmethod
    def publish_active_path(userdata):
        def calc_dist(p1, p2):
            return math.sqrt((p1.position.x - p2.position.x) ** 2 +
                             (p1.position.y - p2.position.y) ** 2)

        def calc_time(stamp):
            return stamp.secs + stamp.nsecs / 1e9

        pub_traj = Path()
        pub_traj.header.frame_id = 'world'

        userdata.new_odom_event.clear()
        userdata.new_odom_event.wait()
        userdata.new_odom_event.clear()
        current_pose = userdata.odom.pose
        current_time = rospy.get_time()
        last_dist = 1
        found_pose = False
        time_diff_sum = 0
        for pose in userdata.trajectory[0].poses:
            if not found_pose:
                dist = calc_dist(current_pose.pose, pose.pose)
                if dist > last_dist:
                    found_pose = True
                else:
                    last_dist = dist
            else:
                time_diff_sum += calc_time(pose.header.stamp)

        pub_traj.header.stamp = rospy.Time.from_sec(current_time + time_diff_sum)

        userdata.trajectory.pop(0)
        userdata.speed.pop(0)

        for path in userdata.trajectory:
            pub_traj.poses += path.poses

        userdata.pub_trajectory.publish(pub_traj)

    def execute(self, userdata):
        cmd_vel = Twist()
        cmd_vel.linear.x = userdata.speed[0]

        # print userdata.robot + " :: brzina segmenta: " + str(userdata.speed[0])

        userdata.pub_speed.publish(cmd_vel)
        userdata.pub_path.publish(userdata.trajectory[0])

        self.publish_active_path(userdata)

        if len(userdata.trajectory) == 0:
            # check if finished goal is also a mission, or a intermidiate goal
            all_goals = userdata.goal_counter[0]
            done_goals = userdata.goal_counter[1]
            active_goals = len(userdata.goal_list)
            if all_goals == done_goals + active_goals:
                # it's a mission
                userdata.goal_counter[1] += 1
                userdata.goal_counter[2].append(rospy.get_time() - userdata.goal_time)

            userdata.goal_list.pop(0)
            return 'goal_reached'
        else:
            return 'next_segment'
