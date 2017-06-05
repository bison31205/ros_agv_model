import rospy
import smach


class GoalAssignment(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['next_goal'],
                             input_keys=['goal_list', 'odom'])

    def execute(self, userdata):
        while len(userdata.goal_list) == 0 or userdata.odom.twist.twist.linear.x != 0:
            pass
        return 'next_goal'
