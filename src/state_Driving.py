import rospy
import smach
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist


class Driving(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['goal_reached', 'next_segment'],
                             input_keys=['robot', 'trajectory', 'segment_index',
                                         'speed', 'max_speed',
                                         'goal_counter', 'goal_list',
                                         'pub_speed', 'pub_path'],
                             output_keys=['trajectory', 'segment_index',
                                          'goal_counter', 'goal_list',
                                          'pub_speed', 'pub_path'])

    def execute(self, userdata):
        cmd_vel = Twist()
        cmd_vel.linear.x = userdata.speed[userdata.segment_index]

        userdata.pub_speed.publish(cmd_vel)
        userdata.pub_path.publish(userdata.trajectory[userdata.segment_index])
        userdata.segment_index += 1

        if userdata.segment_index == len(userdata.trajectory):
            userdata.goal_counter[1] += 1
            userdata.goal_list.pop(0)
            return 'goal_reached'
        else:
            return 'next_segment'
