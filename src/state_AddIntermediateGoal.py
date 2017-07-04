import rospy
import smach


class AddIntermediateGoal(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['goal_added'],
                             input_keys=['map_segments', 'goal_list', 'trajectory'],
                             output_keys=['map_segments', 'goal_list'])

    def execute(self, userdata):
        new_goal = userdata.map_segments.find_intermediate_goal(
            userdata.trajectory[0].poses[0].pose.position.x,
            userdata.trajectory[0].poses[0].pose.position.y,
            userdata.trajectory[-1].poses[-1].pose.position.x,
            userdata.trajectory[-1].poses[-1].pose.position.y,
        )
        userdata.goal_list.insert(0, new_goal)
        return 'goal_added'


