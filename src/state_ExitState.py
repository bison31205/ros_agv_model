import smach

from msg_pkg.msg import Statistics


class ExitState(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['END'],
                             input_keys=['goal_counter', 'pub_exit'],
                             output_keys=['pub_exit'])

    def execute(self, userdata):

        stat_msg = Statistics()
        stat_msg.received_goals = userdata.goal_counter[0]
        stat_msg.completed_goals = userdata.goal_counter[1]
        stat_msg.active_time = sum(userdata.goal_counter[2])

        userdata.pub_exit.publish(stat_msg)

        return 'END'
