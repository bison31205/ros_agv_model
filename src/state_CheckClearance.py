import smach
import rospy
import math
import threading


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
                             input_keys=['trajectory', 'segment_index', 'odom',
                                         'other_robots_trajectories',
                                         'robot_list', 'robot'],
                             output_keys=['trajectory', 'conflict_features', 'robot_conflict'])
        self.stop_thread = False
        self.conflict = False

    @staticmethod
    def check_distance(odom, end):
        dist = math.sqrt((odom.position.x - end.position.x) ** 2 +
                         (odom.position.y - end.position.y) ** 2)

        return True if dist < 0.15 else False

    def check_for_conflicts(self, userdata):
        self.conflict = False
        first_pass_done = False
        while not (self.stop_thread and first_pass_done):
            if not first_pass_done:
                first_pass_done = True
            robot_time_index = dict()
            current_time = (userdata.trajectory[userdata.segment_index].poses[0].header.stamp.secs +
                            userdata.trajectory[userdata.segment_index].poses[0].header.stamp.nsecs / 1e9)
            # find point of trajectories which correspond current time

            for robot in userdata.robot_list:

                for (k, pose) in zip(xrange(len(userdata.other_robots_trajectories[robot].poses)),
                                     userdata.other_robots_trajectories[robot].poses):
                    robot_time = pose.header.stamp.secs + pose.header.stamp.nsecs / 1e9
                    if robot_time >= current_time:
                        robot_time_index[robot] = k
                        break
                    robot_time_index[robot] = len(userdata.other_robots_trajectories[robot].poses) - 1

            while robot_time_index[userdata.robot] < len(userdata.other_robots_trajectories[userdata.robot].poses):
                robot_pose = userdata.other_robots_trajectories[userdata.robot].poses[robot_time_index[userdata.robot]].pose
                for robot in userdata.robot_list:
                    if robot == userdata.robot:
                        continue

                    temp_pose = userdata.other_robots_trajectories[robot].poses[robot_time_index[robot]].pose
                    dist = math.sqrt((robot_pose.position.x - temp_pose.position.x) ** 2 +
                                     (robot_pose.position.y - temp_pose.position.y) ** 2)

                    if dist < 0.5:
                        self.conflict = True
                        print robot_pose.position
                        print temp_pose.position

                        userdata.robot_conflict = robot
                        break

                    if robot_time_index[robot] + 1 < len(userdata.other_robots_trajectories[robot].poses):
                        robot_time_index[robot] += 1

                if self.conflict:
                    break
                robot_time_index[userdata.robot] += 1

    def execute(self, userdata):
        # Start parallel thread which checks for conflicts
        conflict_thread = FuncThread(self.check_for_conflicts, userdata)
        self.stop_thread = False
        conflict_thread.start()

        # Check if we are close enough to publish next segment
        while not self.check_distance(userdata.odom.pose.pose,
                                      userdata.trajectory[userdata.segment_index].poses[0].pose):
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
