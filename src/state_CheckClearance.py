import smach,rospy
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
                             input_keys=['trajectory', 'odom', 'conflict_data',
                                         'robots_trajectories', 'goal_time',
                                         'robot_list', 'robot', 'trajectory_updated_robots',
                                         'trajectory_updated_event',
                                         'new_odom_event'],
                             output_keys=['trajectory', 'conflict_data',
                                          'trajectory_updated_event',
                                          'trajectory_updated_robots',
                                          'new_odom_event'])
        self.stop_thread = False
        self.first_pass_done = False
        self.thread_is_waiting = False
        self.conflict = False

    @staticmethod
    def check_distance(odom, end):
        dist = math.sqrt((odom.position.x - end.position.x) ** 2 +
                         (odom.position.y - end.position.y) ** 2)

        return True if dist < 0.25 else False

    def check_for_conflicts(self, userdata):
        self.conflict = False

        def calc_dist(p1, p2):
            return math.sqrt((p1.pose.position.x - p2.pose.position.x) ** 2 +
                             (p1.pose.position.y - p2.pose.position.y) ** 2)

        def calc_time(stamp):
            return stamp.secs + stamp.nsecs / 1e9

        while not (self.stop_thread and self.first_pass_done):
            # If its first pass, check for conflict with all robots
            if not self.first_pass_done:
                self.first_pass_done = True
                robot_list = userdata.robot_list
            # If first pass is done, only check with robots whose trajectory changed
            else:
                self.thread_is_waiting = True
                userdata.trajectory_updated_event.wait()
                robot_list = userdata.trajectory_updated_robots[:]
                userdata.trajectory_updated_robots = []
                userdata.trajectory_updated_event.clear()
                self.thread_is_waiting = False
                # If event is raised from CheckClearance, return immediately
                if self.stop_thread:
                    return

            # time at the start of next segment
            current_time = (userdata.trajectory[0].poses[0].header.stamp.secs +
                            userdata.trajectory[0].poses[0].header.stamp.nsecs / 1e9)

            index = dict()

            robot1 = userdata.robot
            # robot1 = current robot (userdata.robot)
            # robot2 = some other robot (robot from robot_list)
            #
            # Find conflict where robot2 tries to pass same point
            # in space in a narrow time window when there is robot1
            for pose1 in userdata.robots_trajectories[robot1].poses:
                pose1_time = calc_time(pose1.header.stamp)
                # don't look at poses which are older than current time
                if pose1_time < current_time:
                    continue

                # check with every robot2 if there is a conflict with robot1 in pose1
                for robot2 in robot_list:
                    # skip robot1 in robot_list
                    if robot2 == robot1:
                        continue

                    # Optimization wise, only check poses with timestamp 'just before' pose1
                    # If we haven't found pose2 with sufficient timestamp yet, start from index 0
                    if robot2 not in index:
                        index[robot2] = 0

                    # Iterate through robot2 poses from saved index
                    for pose2 in userdata.robots_trajectories[robot2].poses[index[robot2]:]:
                        pose2_time = calc_time(pose2.header.stamp)

                        # If time difference is bellow 0.1 seconds, check distance
                        if abs(pose1_time - pose2_time) < 0.1:
                            if calc_dist(pose1, pose2) < 0.50:
                                # [robot_name, safe_pose, future_conflict_time, found_safe_pose]
                                userdata.conflict_data = [robot2, pose1, pose1_time, False]
                                index[robot1] = userdata.robots_trajectories[robot1].poses.index(pose1)
                                index[robot2] = userdata.robots_trajectories[robot2].poses.index(pose2)
                                self.conflict = True
                                break
                        # If pose2 is ahead from pose1, stop checking robot2
                        elif pose2_time > pose1_time:
                            break
                        # If pose2 is before pose2, update index
                        else:
                            index[robot2] += 1
                    if self.conflict:
                        break
                if self.conflict:
                    break

            # If there is conflict, check if trajectories overlap constantly
            if self.conflict:
                print robot1
                # Iterate pose by pose and find safe pose1
                # - pose1 iterates reversed from conflict pose to start pose
                # - pose2 iterates from conflict pose until final pose
                robot2 = userdata.conflict_data[0]
                consecutive_safe = 0
                for (pose1, pose2) in zip(reversed(userdata.robots_trajectories[robot1].poses[:index[robot1]]),
                                          userdata.robots_trajectories[robot2].poses[index[robot2]+10:]):

                    if calc_dist(pose1, pose2) < 0.5:
                        consecutive_safe = 0
                    else:
                        # if we found last point of continuous overlap, then return last safe pose
                        # [robot_name, safe_pose, conflict_time, continuous_overlap, found_safe_pose]
                        userdata.conflict_data[1] = pose1
                        userdata.conflict_data[2] = pose2.header.stamp.secs + pose2.header.stamp.nsecs / 1e9
                        userdata.conflict_data[3] = True
                        consecutive_safe += 1
                        if consecutive_safe == 10:
                            break
                if self.conflict and not userdata.conflict_data[3]:
                    userdata.conflict_data[1] = userdata.robots_trajectories[robot1].poses[0]
                    userdata.conflict_data[2] = float('Inf')
        return

    def execute(self, userdata):
        # Start parallel thread which checks for conflicts
        # - first pass is full pass
        # - take another pass only if there is change in other robot trajectory
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
