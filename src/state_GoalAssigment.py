import rospy
import smach


class GoalAssigment(smach.StateMachine):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['get_new_mission', 'go_to_next_goal'],
                             input_keys=['goal_list'],
                             output_keys=['goal_list'])

    def execute(self, userdata):
        userdata.goal_list.pop(0)
        if len(userdata.goal_list):
            return 'go_to_next_goal'
        else:
            return 'get_new_mission'
