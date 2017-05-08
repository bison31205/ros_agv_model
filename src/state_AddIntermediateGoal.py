import rospy
import smach


class AddIntermediateGoal(smach.StateMachine):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['goal_added'],
                             input_keys=['map'],
                             output_keys=['zones'])

    def execute(self, userdata):
        return 'goal_added'


