import smach


class ChangeSpeed(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['speed_changed'],
                             input_keys=['conflict_pose', 'speed',
                                         'trajectory', 'map_segments'],
                             output_keys=['map_segments', 'speed'])

    @staticmethod
    def in_segment(pose_c, pose1, pose2):
        [tx1, tx2] = [pose1.x, pose2.x] if pose1.x < pose2.x \
            else [pose2.x, pose1.x]
        [ty1, ty2] = [pose1.y, pose2.y] if pose1.y < pose2.y \
            else [pose2.y, pose1.y]
        
        if (((pose_c.x > tx1) and (pose_c.x < tx2)) and
                ((pose_c.y > ty1) and (pose_c.y < ty2))):
            return True
        else:
            return False

    def execute(self, userdata):
        index = 0
        best_index = 0
        best_seg_val = 5
        
        for (segment, seg_speed) in zip(userdata.trajectory, userdata.speed):
            position1 = segment.poses[0].pose.position
            position2 = segment.poses[-1].pose.position
            if self.in_segment(userdata.conflict_pose.pose.position, position1, position2):
                break
            
            p1_val = userdata.map_segments.get_segment_value(position1.x, position2.y)
            p2_val = userdata.map_segments.get_segment_value(position1.x, position2.y)
            p_val_max = p1_val if p1_val < p2_val else p2_val

            if p_val_max < best_seg_val:
                best_index = index
                best_seg_val = p_val_max
            # This part of idea is not working as it should currently
            # elif p_val_max == best_seg_val:
            #    if userdata.speed[index] > userdata.speed[best_index]:
            #        best_index = index
            #        best_seg_val = p_val_max

            index += 1

        userdata.speed[best_index] *= 0.75

        return 'speed_changed'

