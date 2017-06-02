import smach
import rospy
import math


class CheckClearance(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['conflict', 'OK_clearance'],
                             input_keys=['trajectory', 'segment_index', 'odom'],
                             output_keys=['trajectory', 'conflict_features'])

    @staticmethod
    def check_distance(odom, end):
        dist = math.sqrt((odom.position.x - end.position.x) ** 2 +
                         (odom.position.y - end.position.y) ** 2)
        return True if dist < 0.05 else False

    def execute(self, userdata):
        while not self.check_distance(userdata.odom.pose.pose,
                                      userdata.trajectory[userdata.segment_index].poses[0].pose):
            pass
        return 'OK_clearance'
