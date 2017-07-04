import math
from geometry_msgs.msg import PoseStamped


class Segment:
    x1 = int()
    x2 = int()
    y1 = int()
    y2 = int()

    def __init__(self):
        pass


class MapSegments:
    def __init__(self):
        self.segments = ["dummy"]
        self.segment_value = dict()
        self.segment_neighbours = dict()
        self.neighbour_sills = dict()
        self.neighbour_sill_quaternions = dict()

    def get_segment_number(self, x, y):
        for segment in self.segments[1:]:
            if (((x > segment.x1) and (x < segment.x2)) and
                    ((y > segment.y1) and (y < segment.y2))):
                return self.segments.index(segment)
        return -1

    def get_segment_value(self, x, y=None):
        if y is None:
            if 0 < x < len(self.segments):
                seg_num = x
            else:
                seg_num = -1
        else:
            seg_num = self.get_segment_number(x, y)
        if seg_num >= 0:
            return self.segment_value[self.segments[seg_num]]
        else:
            return 0

    def get_segment_sills_avg(self, segment_index):
        x = 0
        y = 0
        for neighbour in self.segment_neighbours[segment_index]:
            x += self.neighbour_sills[(segment_index, neighbour)][0]
            y += self.neighbour_sills[(segment_index, neighbour)][1]

        return [x / len(self.segment_neighbours[segment_index]),
                y / len(self.segment_neighbours[segment_index])]

    def add_segment(self, x1, y1, x2, y2, value):
        temp = Segment()
        [temp.x1, temp.x2] = [x1, x2] if x1 < x2 else [x2, x1]
        [temp.y1, temp.y2] = [y1, y2] if y1 < y2 else [y2, y1]

        self.segments.append(temp)
        self.segment_value[temp] = value

    def define_neighbourhood(self, segment, neighbours, sills, sill_quaternions):
        self.segment_neighbours[segment] = neighbours
        for neighbour, sill, quat in zip(neighbours, sills, sill_quaternions):
            self.neighbour_sills[(segment, neighbour)] = sill
            self.neighbour_sill_quaternions[(segment, neighbour)] = quat

    def find_all_paths(self, visited, final):
        found_paths = []
        for seg in self.segment_neighbours[visited[-1]]:
            if seg in visited:
                continue
            visited.append(seg)
            if seg == final:
                found_paths.append(visited[:])
            else:
                found_paths += self.find_all_paths(visited, final)
            visited.pop(-1)

        return found_paths

    def find_intermediate_goal(self, x1, y1, x2, y2):
        path_start_segment = self.get_segment_number(x1, y1)
        path_final_segment = self.get_segment_number(x2, y2)

        all_paths = []
        for seg in self.segment_neighbours[path_start_segment]:
            visited = [seg]
            if seg == path_final_segment:
                all_paths.append(visited)
            else:
                all_paths += self.find_all_paths(visited, path_final_segment)

        def path_length(path):
            def dist(p1, p2):
                return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
            length = dist([x1, y1],
                          self.neighbour_sills[(path_start_segment, path[0])])
            if len(path) > 1:
                path.insert(0, path_start_segment)
                for seg1, seg2, seg3 in zip(path[:-2], path[1:-1], path[2:]):
                    length += dist(self.neighbour_sills[(seg1, seg2)],
                                   self.neighbour_sills[(seg2, seg1)])
                    length += dist(self.neighbour_sills[(seg2, seg1)],
                                   self.neighbour_sills[(seg2, seg3)])
                path.pop(0)
                length += dist(self.neighbour_sills[(path[-2], path_final_segment)],
                               self.neighbour_sills[(path_final_segment, path[-2])])
                length += dist([x2, y2],
                               self.neighbour_sills[(path_final_segment, path[-2])])
            else:
                length += dist(self.neighbour_sills[(path_start_segment,
                                                     path_final_segment)],
                               self.neighbour_sills[(path_final_segment,
                                                     path_start_segment)])
                length += dist([x2, y2],
                               self.neighbour_sills[(path_final_segment,
                                                     path_start_segment)])

            return length

        all_paths.sort(key=path_length)
        pose = PoseStamped()
        if len(all_paths) == 1:
            [x, y] = self.get_segment_sills_avg(all_paths[0][0])
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation = self.neighbour_sill_quaternions[(all_paths[0][0], path_start_segment)]
        elif all_paths[1][1] == path_start_segment and self.get_segment_value(path_start_segment) == 1:
            [x, y] = self.get_segment_sills_avg(path_start_segment)
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation = self.neighbour_sill_quaternions[(path_start_segment, all_paths[1][2])]

        else:
            pose.pose.position.x = self.neighbour_sills[(path_start_segment, all_paths[1][0])][0]
            pose.pose.position.y = self.neighbour_sills[(path_start_segment, all_paths[1][0])][1]
            pose.pose.orientation = self.neighbour_sill_quaternions[(path_start_segment, all_paths[1][0])]

        return pose
