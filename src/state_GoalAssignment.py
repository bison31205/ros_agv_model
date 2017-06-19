import rospy
import smach


class GoalAssignment(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['next_goal', 'no_new_goals'],
                             input_keys=['goal_list', 'odom', 'goal_counter'],
                             output_keys=['goal_time'])
        self.goal_index = -1

    def execute(self, userdata):
        # wait for robot to stop (velocity = 0)
        while userdata.odom.twist.twist.linear.x != 0:
            pass

        # wait for new missions if none are available
        waiting_time_start = rospy.get_time()
        while len(userdata.goal_list) == 0:
            if (rospy.get_time() - waiting_time_start) > 60:  # in seconds
                return 'no_new_goals'
            pass

        # reset time counter if we completed mission
        if self.goal_index != userdata.goal_counter[1]:
            userdata.goal_time = rospy.get_time()
            self.goal_index += 1
        return 'next_goal'
