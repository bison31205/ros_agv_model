import smach
import math

from msg_pkg.msg import Features


class ConflictResolver(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['just_drive', 'change_speed', 'change_path', 'NaN'],
                             input_keys=['robot', 'conflict_data', 'goal_time',
                                         'robot_data', 'robots_features',
                                         'odom', 'trajectory', 'map_zones',
                                         'new_odom_event', 'pub_features',
                                         'features_updated_robots',
                                         'features_updated_event'],
                             output_keys=['new_odom_event', 'pub_features', 'map_zones',
                                          'features_updated_event', 'features_updated_robots'])

    @staticmethod
    def n_sphere(feat_1, feat_2, weight, param):
        dist = 0
        for f1, f2, w, p in zip(feat_1, feat_2, weight[1:], param[1:]):
            dist += ((f1 - f2) * w - p) ** 2
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

    @staticmethod
    def publish_features(userdata):
        # [Current mission duration
        #  Current pose map zone type
        #  Average map zone type from current pose to conflict pose
        # ]
        current_pose = userdata.trajectory[0].poses[0]
        conflict_pose = userdata.conflict_data[1]
        current_time = (current_pose.header.stamp.secs +
                        current_pose.header.stamp.nsecs / 1e9)

        current_zone = userdata.map_zones.get_zone_number(current_pose.pose.position.x,
                                                          current_pose.pose.position.y)

        current_zone_value = userdata.map_zones.get_zone_value(current_zone)
        average_zone_value = userdata.map_zones.get_zone_value(current_zone)

        num_of_seg = 0
        for segment in userdata.trajectory:
            num_of_seg += 1
            average_zone_value += userdata.map_zones.get_zone_value(segment)
            if conflict_pose in segment:
                break
        average_zone_value /= num_of_seg

        features = Features()
        features.features += [current_time - userdata.goal_time,
                              current_zone_value,
                              average_zone_value]

        userdata.pub_features.publish(features)

    def execute(self, userdata):
        # # # # [robot_name, safe_pose, conflict_time, continuous_overlap]
        # print userdata.conflict_data

        # Wait for updated features from robot1 and robot2
        # Publish robot1 vector until communication is over
        # - both robots will publish their own data until both of them have both robot features
        userdata.features_updated_robots = []
        while not (userdata.robot in userdata.features_updated_robots and
                   userdata.conflict_data[0] in userdata.features_updated_robots):
            # Update and publish features vector
            self.publish_features(userdata)
            userdata.features_updated_event.wait()
            userdata.features_updated_event.clear()

        best_outcome = 'NaN'
        best_dist = 0
        for outcome in userdata.robot_data:

            new_dist = self.n_sphere(userdata.robots_features[userdata.robot].features,
                                     userdata.robots_features[userdata.conflict_data[0]].features,
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
                userdata.new_odom_event.wait()
                userdata.new_odom_event.clear()
            return 'just_drive'
        else:
            if userdata.map_zones.get_zonet_value(userdata.conflict_data[1].pose.position.x,
                                                  userdata.conflict_data[1].pose.position.y) == 1:
                return 'change_speed'
            else:
                return 'change_path'

