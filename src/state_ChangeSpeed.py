import smach


class ChangeSpeed(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['speed_changed'],
                             input_keys=['conflict_data', 'speed',
                                         'segment_time', 'max_speed',
                                         'trajectory', 'map_zones'],
                             output_keys=['map_zones', 'speed'])

    def execute(self, userdata):
        index = 0
        best_index = [0]
        best_seg_val = 5

        # Find most suitable segments of trajectory to change speed
        for (segment, seg_speed) in zip(userdata.trajectory, userdata.speed):
            # stop searching if we reached segment where safe pose is
            if userdata.conflict_data[1] in segment.poses:
                break

            seg_start_value = userdata.map_zones.get_zone_value(segment.poses[0].pose.position.x,
                                                                segment.poses[0].pose.position.y)
            seg_end_value = userdata.map_zones.get_zone_value(segment.poses[-1].pose.position.x,
                                                              segment.poses[-1].pose.position.y)
            seg_val = seg_start_value if seg_start_value > seg_end_value else seg_end_value

            if seg_val < best_seg_val:
                best_index = [index]
                best_seg_val = seg_val
            elif seg_val == best_seg_val:
                best_index.append(index)
            index += 1

        # Calculate speed modifier
        safe_pose_time = (userdata.conflict_data[1].header.stamp.secs +
                          userdata.conflict_data[1].header.stamp.nsecs / 1e9)
        needed_time = userdata.conflict_data[2] - safe_pose_time

        best_index_time = 0
        for index in best_index:
            best_index_time += userdata.segment_time * userdata.max_speed / userdata.speed[index]

        for index in best_index:
            userdata.speed[index] *= best_index_time / (5 + needed_time + best_index_time)

        return 'speed_changed'

