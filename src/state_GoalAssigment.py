import rospy
import smach


class GoalAssigment(smach.StateMachine):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['new_mission', 'next_goal'],
                             input_keys=['goal_list'],
                             output_keys=['goal_list'])

    def execute(self, userdata):
        userdata.goal_list.pop(0)
        if len(userdata.goal_list):
            return 'next_goal'
        else:
            return 'new_mission'
