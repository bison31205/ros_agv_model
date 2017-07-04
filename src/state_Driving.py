import rospy
import smach
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist


class Driving(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['goal_reached', 'next_segment'],
                             input_keys=['robot', 'trajectory',
                                         'speed', 'max_speed',
                                         'goal_counter', 'goal_list', 'goal_time',
                                         'pub_speed', 'pub_path', 'pub_trajectory'],
                             output_keys=['trajectory', 'speed'
                                          'goal_counter', 'goal_list',
                                          'pub_speed', 'pub_path', 'pub_trajectory'])

    @staticmethod
    def publish_active_path(userdata):
        pub_traj = Path()
        pub_traj.header.frame_id = 'world'
        for path in userdata.trajectory:
            pub_traj.poses += path.poses

        userdata.pub_trajectory.publish(pub_traj)

    def execute(self, userdata):
        cmd_vel = Twist()
        cmd_vel.linear.x = userdata.speed[0]

        # print userdata.robot + " :: brzina segmenta: " + str(userdata.speed[0])

        userdata.pub_speed.publish(cmd_vel)
        userdata.pub_path.publish(userdata.trajectory[0])

        userdata.trajectory.pop(0)
        userdata.speed.pop(0)

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
