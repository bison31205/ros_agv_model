import smach


class ExitState(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['END'],
                             input_keys=['robot', 'robot_conflict',
                                         'robot_data', 'robots_features'],
                             output_keys=['zones'])

    def execute(self, userdata):
        return 'END'
