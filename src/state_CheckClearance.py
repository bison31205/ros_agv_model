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
                             input_keys=['trajectory', 'odom', 'conflict_data',
                                         'robots_trajectories', 'goal_time',
                                         'robot_list', 'robot', 'pub_features',
                                         'map_segments', 'trajectory_updated_robot',
                                         'trajectory_updated_event',
                                         'new_odom_event'],
                             output_keys=['trajectory', 'conflict_data',
                                          'pub_features', 'map_segments',
                                          'trajectory_updated_event',
                                          'new_odom_event'])
        self.stop_thread = False
        self.first_pass_done = False
        self.thread_is_waiting = False
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
            # If its first pass, check for conflict with all robots
            if not self.first_pass_done:
                self.first_pass_done = True
                robot_list = userdata.robot_list
            # If first pass is done, only check with robots whose trajectory changed
            else:
                self.thread_is_waiting = True
                userdata.trajectory_updated_event.wait()
                userdata.trajectory_updated_event.clear()
                self.thread_is_waiting = False
                # If event is raised from CheckClearance, return immediately
                if self.stop_thread:
                    return
                # Else, write which robot has updated trajectory
                robot_list = [userdata.trajectory_updated_robot]

            current_time = (userdata.trajectory[0].poses[0].header.stamp.secs +
                            userdata.trajectory[0].poses[0].header.stamp.nsecs / 1e9)

            old_index = dict()

            # Find conflict where two robots try to pass same point in space in a narrow time window
            for pose1 in userdata.robots_trajectories[userdata.robot].poses:
                pose1_time = pose1.header.stamp.secs + pose1.header.stamp.nsecs / 1e9
                # don't look at poses which are older than current time
                if pose1_time < current_time:
                    continue

                # check with every robot if there is a conflict in 'pose'
                for robot in robot_list:
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
                                # [robot_name, safe_pose, conflict_time, continuous_overlap]
                                userdata.conflict_data = [robot, pose1, pose1_time, False]
                                old_index[robot] = index
                                old_index[userdata.robot] = userdata.robots_trajectories[userdata.robot].poses.index(
                                    pose1)
                                self.conflict = True
                                break

                        elif pose2_time > pose1_time:
                            break
                        else:
                            old_index[robot] = index
                            index += 1
                    if self.conflict:
                        break
                if self.conflict:
                    break

            # If there is conflict, check if trajectories overlap constantly
            if self.conflict:
                for (pose1, pose2) in zip(reversed(userdata.robots_trajectories[userdata.robot].poses[:old_index[userdata.robot]]),
                                          userdata.robots_trajectories[userdata.conflict_data[0]].poses[old_index[userdata.conflict_data[0]] + 10:]):
                    dist = math.sqrt((pose1.pose.position.x - pose2.pose.position.x) ** 2 +
                                     (pose1.pose.position.y - pose2.pose.position.y) ** 2)
                    if dist < 0.5:
                        # [robot_name, safe_pose,  conflict_time, continuous_overlap]
                        userdata.conflict_data[3] = True
                    else:
                        # if we found last point of continuous overlap, then return last safe pose
                        # [robot_name, safe_pose, conflict_time, continuous_overlap]
                        userdata.conflict_data[1] = pose1
                        userdata.conflict_data[2] = pose2.header.stamp.secs + pose2.header.stamp.nsecs / 1e9
                        return
            # return if there is no conflict
            return

    def execute(self, userdata):
        # Update and publish features vector
        self.publish_features(userdata)

        # Start parallel thread which checks for conflicts
        # - first pass is full pass
        # - take another pass only if there is change in other trajectory
        conflict_thread = FuncThread(self.check_for_conflicts, userdata)
        self.stop_thread = False
        self.first_pass_done = False
        conflict_thread.start()

        # Wait while we are close enough to publish next segment
        while not self.check_distance(userdata.odom.pose.pose,
                                      userdata.trajectory[0].poses[0].pose):
            # Break if there is a conflict
            if self.conflict:
                break
            # Idle while waiting for new odom data
            userdata.new_odom_event.wait()
            userdata.new_odom_event.clear()

        # Stop conflict check (at least 1 pass)
        self.stop_thread = True
        if self.thread_is_waiting:
            # False event call to un-pause the thread
            userdata.trajectory_updated_event.set()
        conflict_thread.join()

        # If there is conflict, resolve it, otherwise publish next segment
        if self.conflict:
            return 'conflict'
        else:
            return 'OK_clearance'
