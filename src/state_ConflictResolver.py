import smach
import math


class ConflictResolver(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['just_drive', 'change_speed', 'change_path', 'NaN'],
                             input_keys=['robot', 'robot_conflict',
                                         'robot_data', 'robots_features'],
                             output_keys=['zones'])

    @staticmethod
    def n_sphere(feat_1, feat_2, weight, param):
        dist = 0
        for f1, f2, w, p in zip(feat_1, feat_2, weight[1:], param[1:]):
            dist += ((f1 - f2)* w - p) ** 2
        dist = math.sqrt(dist)
        if dist <= param[0]:
            return weight[0] * dist / param[0]
        else:
            return 'NaN'

    def execute(self, userdata):
        print userdata.robot_conflict
        best_outcome = 'NaN'
        best_dist = 0
        for outcome in userdata.robot_data:
            new_dist = self.n_sphere(userdata.robots_features[userdata.robot].features,
                                     userdata.robots_features[userdata.robot_conflict].features,
                                     userdata.robot_data[outcome]["weight"],
                                     userdata.robot_data[outcome]["param"])
            if new_dist == 'NaN':
                pass
            elif best_outcome == 'Nan' or new_dist < best_dist:
                    best_outcome = outcome
                    best_dist = new_dist

        return best_outcome

