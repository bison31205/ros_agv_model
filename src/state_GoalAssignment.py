import rospy
import smach


class GoalAssignment(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['next_goal', 'no_new_goals'],
                             input_keys=['goal_list', 'odom', 'goal_counter',
                                         'new_odom_event', 'new_mission_event'],
                             output_keys=['goal_time', 'new_odom_event',
                                          'new_mission_event'])

    def execute(self, userdata):
        # wait for robot to stop (velocity = 0)
        while userdata.odom.twist.twist.linear.x != 0:
            userdata.new_odom_event.wait()
            userdata.new_odom_event.clear()
            pass

        # wait for new missions if none are available
        if len(userdata.goal_list) == 0:
            if not userdata.new_mission_event.wait(
                    None if userdata.goal_counter[0] == 0 else 10
            ):
                return 'no_new_goals'
            userdata.new_mission_event.clear()

        # reset time counter if next goal is also a mission
        all_goals = userdata.goal_counter[0]
        done_goals = userdata.goal_counter[1]
        active_goals = len(userdata.goal_list)
        if all_goals == done_goals + active_goals:
            userdata.goal_time = rospy.get_time()

        return 'next_goal'
