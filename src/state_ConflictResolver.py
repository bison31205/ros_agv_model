import rospy
import smach


def n_sphere(self, feat, weight, param):
    dist = 0
    for f, w, p in itertools.izip(feat[1:], weight[1:], param[1:]):
        dist += (f * w - p) ** 2
    dist = math.sqrt(dist)
    if dist <= param[0]:
        return weight[0] * dist / param[0]
    else:
        return 'NaN'


class ConflictResolver(smach.StateMachine):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['just_drive', 'change_speed', 'change_path'],
                             input_keys=['map'],
                             output_keys=['zones'])

    def execute(self, userdata):
        return 'just_drive'
