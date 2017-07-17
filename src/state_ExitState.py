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
        stat_msg.goals_received = userdata.goal_counter[0]
        stat_msg.goals_completed = userdata.goal_counter[1]
        stat_msg.ideal_time = userdata.goal_counter[2]
        stat_msg.real_time = userdata.goal_counter[3]
        stat_msg.ideal_path = userdata.goal_counter[4]
        stat_msg.real_path = userdata.goal_counter[5]
        stat_msg.goal_started = userdata.goal_counter[6]
        stat_msg.goal_finished = userdata.goal_counter[7]

        userdata.pub_exit.publish(stat_msg)

        return 'END'
