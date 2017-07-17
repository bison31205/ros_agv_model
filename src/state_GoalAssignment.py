import rospy
import smach


class GoalAssignment(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['next_goal', 'no_new_goals'],
                             input_keys=['goal_list', 'odom', 'goal_counter', 'new_mission',
                                         'new_odom_event', 'new_mission_event'],
                             output_keys=['goal_time', 'new_odom_event',
                                          'new_mission_event', 'new_mission'])
        self.intermediate_goal = False

    def execute(self, userdata):
        # wait for robot to stop (velocity = 0)
        while userdata.odom.twist.twist.linear.x != 0:
            userdata.new_odom_event.wait()
            userdata.new_odom_event.clear()
            pass

        if not userdata.goal_counter[0] == 0:
            all_goals = userdata.goal_counter[0]
            done_goals = userdata.goal_counter[1]
            active_goals = len(userdata.goal_list)
            if all_goals == done_goals + active_goals:
                if not self.intermediate_goal:
                    userdata.goal_counter[7].append(rospy.get_time())

        # wait for new missions if none are available
        if len(userdata.goal_list) == 0:
            userdata.new_mission_event.clear()
            if not userdata.new_mission_event.wait(
                    # if its first mission, wait as long as it takes
                    # if there were some mission before, wait for 5 seconds
                    None if userdata.goal_counter[0] == 0 else 5
            ):
                # exit if timeout is reached
                return 'no_new_goals'
            userdata.new_mission_event.clear()

        # reset time counter if next goal is also a mission
        all_goals = userdata.goal_counter[0]
        done_goals = userdata.goal_counter[1]
        active_goals = len(userdata.goal_list)
        if all_goals == done_goals + active_goals:
            if not self.intermediate_goal:
                userdata.goal_time = rospy.get_time()
                userdata.goal_counter[6].append(rospy.get_time())
                userdata.new_mission = True
            else:
                self.intermediate_goal = False
                userdata.new_mission = False
        else:
            self.intermediate_goal = True
            userdata.new_mission = False

        return 'next_goal'
