import rospy
import smach
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist


class Driving(smach.StateMachine):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['goal_reached', 'next_segment'],
                             input_keys=['trajectory', 'segment_index', 'speed', 'max_speed'],
                             output_keys=['trajectory', 'segment_index'])

    def execute(self, userdata):
        speed_pub = rospy.Publisher(userdata.robot + "/cmd_vel", Twist, queue_size=10)
        path_pub = rospy.Publisher(userdata.robot + "/follow_path", Path, queue_size=10)

        cmd_vel = Twist()
        cmd_vel.linear.x = userdata.speed

        speed_pub.publish(cmd_vel)
        path_pub.publish(userdata.trajectory[userdata.segment_index])
        userdata.segment_index += 1

        if userdata.segment_index == len(userdata.trajectory):
            return 'goal_reached'
        else:
            return 'next_segment'
