import smach, rospy
import math

from msg_pkg.msg import Features


class ConflictResolver(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['continue', 'change_speed', 'change_path',
                                       'recalculate_trajectory', 'NaN'],
                             input_keys=['robot', 'conflict_data', 'goal_time',
                                         'robot_data', 'robots_features',
                                         'odom', 'trajectory', 'map_zones',
                                         'new_odom_event', 'pub_features',
                                         'features_updated_robots', 'segment_time',
                                         'features_updated_event',
                                         'ignore_conflict_robots'],
                             output_keys=['new_odom_event', 'pub_features', 'map_zones',
                                          'features_updated_event', 'features_updated_robots',
                                          'ignore_conflict_robots'])

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

        return True if dist < 0.25 else False

    @staticmethod
    def publish_features(userdata):
        # 1 # Current mission duration
        # 2 # Current pose map zone type
        # 3 # Average map zone value from current pose to conflict pose
        # 4 # Average map zone value for whole trajectory
        # 5 # Distance to conflict pose
        # 6 # Distance to safe pose

        current_pose = userdata.trajectory[0].poses[0]
        conf_pose = userdata.conflict_data[1]
        safe_pose = userdata.conflict_data[3]

        # 1
        current_time = (current_pose.header.stamp.secs +
                        current_pose.header.stamp.nsecs / 1e9)

        # 2
        current_zone = userdata.map_zones.get_zone_number(current_pose.pose.position.x,
                                                          current_pose.pose.position.y)

        current_zone_value = userdata.map_zones.get_zone_value(current_zone)

        # 3 & 4
        avg_zone_val = 0
        avg_zone_val_conf = 0
        num_of_seg_conf = 0
        safe_pose_reached = False
        for segment in userdata.trajectory:
            seg_start_value = userdata.map_zones.get_zone_value(segment.poses[0].pose.position.x,
                                                                segment.poses[0].pose.position.y)
            seg_end_value = userdata.map_zones.get_zone_value(segment.poses[-1].pose.position.x,
                                                              segment.poses[-1].pose.position.y)

            avg_zone_val += seg_start_value if seg_start_value > seg_end_value else seg_end_value

            if not safe_pose_reached:
                avg_zone_val_conf += seg_start_value if seg_start_value > seg_end_value else seg_end_value
                num_of_seg_conf += 1.0

            if safe_pose in segment.poses:
                break

        avg_zone_val_conf /= num_of_seg_conf
        avg_zone_val /= float(len(userdata.trajectory))

        # 5 & 6
        dist_conf_pose = math.sqrt((current_pose.pose.position.x - conf_pose.pose.position.x) ** 2 +
                                   (current_pose.pose.position.y - conf_pose.pose.position.y) ** 2)

        dist_safe_pose = math.sqrt((current_pose.pose.position.x - safe_pose.pose.position.x) ** 2 +
                                   (current_pose.pose.position.y - safe_pose.pose.position.y) ** 2)

        features = Features()
        features.features += [current_time - userdata.goal_time,
                              current_zone_value,
                              avg_zone_val_conf,
                              avg_zone_val,
                              dist_conf_pose,
                              dist_safe_pose]

        userdata.pub_features.publish(features)

    def execute(self, userdata):
        # # # # [robot_name, safe_pose, conflict_time, continuous_overlap]
        # print userdata.conflict_data

        # Wait for updated features from robot1 and robot2
        # Publish robot1 vector until communication is over
        # - both robots will publish their own data until both of them have both robot features
        userdata.features_updated_robots = []
        waiting_start = rospy.get_time()
        while not (userdata.robot in userdata.features_updated_robots and
                   userdata.conflict_data[0] in userdata.features_updated_robots):
            # Update and publish features vector
            self.publish_features(userdata)
            userdata.features_updated_event.wait()
            userdata.features_updated_event.clear()
            # If only this robot detected the conflict, ignore it for now and recalculate trajectory
            if rospy.get_time() - waiting_start > userdata.segment_time:
                return 'recalculate_trajectory'
        self.publish_features(userdata)

        best_outcome = 'NaN'
        best_dist = 0
        for outcome in userdata.robot_data:
            new_dist = self.n_sphere(userdata.robots_features[userdata.robot].features,
                                     userdata.robots_features[userdata.conflict_data[0]].features,
                                     userdata.robot_data[outcome]["weight"],
                                     userdata.robot_data[outcome]["param"])
            print userdata.robot, outcome, new_dist
            if new_dist == 'NaN':
                pass
            elif best_outcome == 'NaN' or new_dist < best_dist:
                    best_outcome = outcome
                    best_dist = new_dist

        print userdata.robot, best_outcome, best_dist

        if best_outcome == 'continue':
            userdata.ignore_conflict_robots.append(userdata.conflict_data[0])

        if best_outcome == 'change_speed':
            if not userdata.conflict_data[4]:
                return 'change_path'

        return best_outcome
