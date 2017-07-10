import smach
import math
from geometry_msgs.msg import PoseStamped


class AddIntermediateGoal(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['goal_added'],
                             input_keys=['map_zones', 'goal_list', 'goal_counter',
                                         'trajectory', 'conflict_data'],
                             output_keys=['map_zones', 'goal_list'])

    def execute(self, userdata):
        def add_intermediate_goal(goal):
            all_goals = userdata.goal_counter[0]
            done_goals = userdata.goal_counter[1]
            active_goals = len(userdata.goal_list)
            # remove old intermediate goal (if it exists)
            if not all_goals == done_goals + active_goals:
                userdata.goal_list.pop(0)
            userdata.goal_list.insert(0, goal)

        # Get all paths to goal with origins in neighbouring zones
        # Paths are sorted by length (firs path corresponds with path from path_planner)
        all_paths = userdata.map_zones.find_intermediate_goal(
            userdata.trajectory[0].poses[0].pose.position.x,
            userdata.trajectory[0].poses[0].pose.position.y,
            userdata.trajectory[-1].poses[-1].pose.position.x,
            userdata.trajectory[-1].poses[-1].pose.position.y
        )
        current_position = userdata.trajectory[0].poses[0].pose.position
        current_zone = userdata.map_zones.get_zone_number(
            current_position.x,
            current_position.y
        )

        new_goal = PoseStamped()
        # If there is only one path, it means that goal is in same
        # zone as robot, and zone has only one neighbour
        # - go outside from this zone
        if len(all_paths) == 1:
            [x, y] = userdata.map_zones.get_zone_sills_avg(all_paths[0][0])
            new_goal.pose.position.x = x
            new_goal.pose.position.y = y
            new_goal.pose.orientation = userdata.map_zones.get_neighbour_sill_quaternion((all_paths[0][0],
                                                                                          current_zone))
            add_intermediate_goal(new_goal)
            return 'goal_added'
        # If second best path is also through starting zone, and starting zone has value 1
        # - do a evasion maneuver
        elif all_paths[1][1] == current_zone and userdata.map_zones.get_zone_value(current_zone) == 1:
            [x, y] = userdata.map_zones.get_zone_sills_avg(current_zone)
            dist = math.sqrt((x - current_position.x) ** 2 +
                             (y - current_position.y) ** 2)
            if not (userdata.conflict_data[3] and dist < 1):
                new_goal.pose.position.x = x
                new_goal.pose.position.y = y
                new_goal.pose.orientation = userdata.map_zones.get_neighbour_sill_quaternion((current_zone,
                                                                                              all_paths[0][0]))
                add_intermediate_goal(new_goal)
                return 'goal_added'
        # Else take the alternative path (it could still be same path,
        # but robot will first visit neighbouring zone)
        new_goal.pose.position.x = userdata.map_zones.get_neighbour_sill((all_paths[1][0], all_paths[1][1]))[0]
        new_goal.pose.position.y = userdata.map_zones.get_neighbour_sill((all_paths[1][0], all_paths[1][1]))[1]
        new_goal.pose.orientation = userdata.map_zones.get_neighbour_sill_quaternion((all_paths[1][0],
                                                                                      all_paths[1][1]))

        add_intermediate_goal(new_goal)
        return 'goal_added'
