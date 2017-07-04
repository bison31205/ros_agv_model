import rospy
import smach


class GoalAssignment(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['next_goal', 'no_new_goals'],
                             input_keys=['goal_list', 'odom', 'goal_counter'],
                             output_keys=['goal_time'])

    def execute(self, userdata):
        # wait for robot to stop (velocity = 0)
        while userdata.odom.twist.twist.linear.x != 0:
            pass

        # wait for new missions if none are available
        waiting_time_start = rospy.get_time()
        while len(userdata.goal_list) == 0:
            if userdata.goal_counter[0] != 0 and (rospy.get_time() - waiting_time_start) > 60:  # in seconds
                return 'no_new_goals'
            pass

        # reset time counter if next goal is also a mission
        all_goals = userdata.goal_counter[0]
        done_goals = userdata.goal_counter[1]
        active_goals = len(userdata.goal_list)
        if all_goals == done_goals + active_goals:
            userdata.goal_time = rospy.get_time()

        return 'next_goal'
