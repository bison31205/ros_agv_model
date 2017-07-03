import smach
import math


class ConflictResolver(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['just_drive', 'change_speed', 'change_path', 'NaN'],
                             input_keys=['robot', 'robot_conflict', 'conflict_pose',
                                         'robot_data', 'robots_features',
                                  'odom', 'trajectory'],
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

    @staticmethod
    def check_distance(odom, end):
        dist = math.sqrt((odom.position.x - end.position.x) ** 2 +
                         (odom.position.y - end.position.y) ** 2)

        return True if dist < 0.15 else False

    def execute(self, userdata):
        # print userdata.robot_conflict, userdata.conflict_pose
        best_outcome = 'NaN'
        best_dist = 0
        for outcome in userdata.robot_data:

            new_dist = self.n_sphere(userdata.robots_features[userdata.robot].features,
                                     userdata.robots_features[userdata.robot_conflict].features,
                                     userdata.robot_data[outcome]["weight"],
                                     userdata.robot_data[outcome]["param"])
            if new_dist == 'NaN':
                pass
            elif best_outcome == 'NaN' or new_dist < best_dist:
                    best_outcome = outcome
                    best_dist = new_dist


# ZA TESTIRRANJE
        if userdata.robot == "mirko":
            while not self.check_distance(userdata.odom.pose.pose,
                                          userdata.trajectory[0].poses[0].pose):
                pass
            return 'just_drive'
        else:
            return 'change_speed'

