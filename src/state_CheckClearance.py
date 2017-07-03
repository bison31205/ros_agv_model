import smach
import math
import threading

from msg_pkg.msg import Features


class FuncThread(threading.Thread):
    def __init__(self, target, *args):
        self._target = target
        self._args = args
        threading.Thread.__init__(self)

    def run(self):
        self._target(*self._args)


class CheckClearance(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['conflict', 'OK_clearance'],
                             input_keys=['trajectory', 'odom',
                                         'robots_trajectories', 'goal_time',
                                         'robot_list', 'robot', 'pub_features',
                                         'map_segments'],
                             output_keys=['trajectory', 'conflict_pose',
                                          'robot_conflict', 'pub_features',
                                          'map_segments'])
        self.stop_thread = False
        self.first_pass_done = False
        self.conflict = False

    @staticmethod
    def check_distance(odom, end):
        dist = math.sqrt((odom.position.x - end.position.x) ** 2 +
                         (odom.position.y - end.position.y) ** 2)

        return True if dist < 0.15 else False

    @staticmethod
    def publish_features(userdata):
        # [Current goal time
        #  Current map segment type
        #  Average map segment type]
        current_pose = userdata.trajectory[0].poses[0]
        current_time = (current_pose.header.stamp.secs +
                        current_pose.header.stamp.nsecs / 1e9)

        current_segment = userdata.map_segments.get_segment_value(current_pose.pose.position.x,
                                                                  current_pose.pose.position.y)

        features = Features()
        features.features += [current_time - userdata.goal_time,  # Current goal time
                              current_segment]  # Current map segment type

        userdata.pub_features.publish(features)

    def check_for_conflicts(self, userdata):
        self.conflict = False

        while not (self.stop_thread and self.first_pass_done):
            if not self.first_pass_done:
                self.first_pass_done = True

            current_time = (userdata.trajectory[0].poses[0].header.stamp.secs +
                            userdata.trajectory[0].poses[0].header.stamp.nsecs / 1e9)

            old_index = dict()

            for pose1 in userdata.robots_trajectories[userdata.robot].poses:
                pose1_time = pose1.header.stamp.secs + pose1.header.stamp.nsecs / 1e9
                # don't look at poses which are older than current time
                if pose1_time < current_time:
                    continue

                # check with every robot if there is a conflict in 'pose'
                for robot in userdata.robot_list:
                    if robot == userdata.robot:
                        continue

                    if robot not in old_index:
                        old_index[robot] = 0

                    index = old_index[robot]

                    for pose2 in userdata.robots_trajectories[robot].poses[old_index[robot]:]:
                        pose2_time = pose2.header.stamp.secs + pose2.header.stamp.nsecs / 1e9

                        if abs(pose1_time - pose2_time) < 0.1:
                            dist = math.sqrt((pose1.pose.position.x - pose2.pose.position.x) ** 2 +
                                             (pose1.pose.position.y - pose2.pose.position.y) ** 2)
                            if dist < 0.5:
                                # print pose1.pose.position
                                # print pose2.pose.position

                                userdata.robot_conflict = robot
                                userdata.conflict_pose = pose1
                                self.conflict = True
                                return

                        elif pose2_time > pose1_time:
                            break
                        else:
                            old_index[robot] = index
                            index += 1

    def execute(self, userdata):
        # Update and publish features vector
        self.publish_features(userdata)

        # Start parallel thread which checks for conflicts
        conflict_thread = FuncThread(self.check_for_conflicts, userdata)
        self.stop_thread = False
        self.first_pass_done = False
        conflict_thread.start()

        # Check if we are close enough to publish next segment
        while not self.check_distance(userdata.odom.pose.pose,
                                      userdata.trajectory[0].poses[0].pose):
            if self.conflict:
                break

        # Stop conflict check (at least 1 pass)
        self.stop_thread = True
        conflict_thread.join()

        # If there is conflict, resolve it, otherwise publish next segment
        if self.conflict:
            return 'conflict'
        else:
            return 'OK_clearance'
