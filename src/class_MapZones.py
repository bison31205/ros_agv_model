import math


class Zone:
    x1 = int()
    x2 = int()
    y1 = int()
    y2 = int()

    def __init__(self):
        pass


class MapZones:
    def __init__(self):
        self.zones = ["dummy"]
        self.zone_value = dict()
        self.zone_neighbours = dict()
        self.neighbour_sills = dict()
        self.neighbour_sill_quaternions = dict()

    def get_zone_number(self, x, y):
        for zone in self.zones[1:]:
            if (((x > zone.x1) and (x < zone.x2)) and
                    ((y > zone.y1) and (y < zone.y2))):
                return self.zones.index(zone)
        return -1

    def get_zone_value(self, x, y=None):
        if y is None:
            if 0 < x < len(self.zones):
                seg_num = x
            else:
                seg_num = -1
        else:
            seg_num = self.get_zone_number(x, y)
        if seg_num >= 0:
            return self.zone_value[self.zones[seg_num]]
        else:
            return 0

    def get_neighbour_sill(self, neighbours):
        return self.neighbour_sills[neighbours]

    def get_neighbour_sill_quaternion(self, neighbours):
        return self.neighbour_sill_quaternions[neighbours]

    def get_zone_sills_avg(self, zone_index):
        x = 0
        y = 0
        for neighbour in self.zone_neighbours[zone_index]:
            x += self.neighbour_sills[(zone_index, neighbour)][0]
            y += self.neighbour_sills[(zone_index, neighbour)][1]

        return [x / len(self.zone_neighbours[zone_index]),
                y / len(self.zone_neighbours[zone_index])]

    def add_zone(self, x1, y1, x2, y2, value):
        temp = Zone()
        [temp.x1, temp.x2] = [x1, x2] if x1 < x2 else [x2, x1]
        [temp.y1, temp.y2] = [y1, y2] if y1 < y2 else [y2, y1]

        self.zones.append(temp)
        self.zone_value[temp] = value

    def define_neighbourhood(self, zone, neighbours, sills, sill_quaternions):
        self.zone_neighbours[zone] = neighbours
        for neighbour, sill, quat in zip(neighbours, sills, sill_quaternions):
            self.neighbour_sills[(zone, neighbour)] = sill
            self.neighbour_sill_quaternions[(zone, neighbour)] = quat

    def find_all_paths(self, visited, final):
        found_paths = []
        for seg in self.zone_neighbours[visited[-1]]:
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
        path_start_zone = self.get_zone_number(x1, y1)
        path_final_zone = self.get_zone_number(x2, y2)

        all_paths = []
        for seg in self.zone_neighbours[path_start_zone]:
            visited = [seg]
            if seg == path_final_zone:
                all_paths.append(visited)
            else:
                all_paths += self.find_all_paths(visited, path_final_zone)

        def path_length(path):
            def dist(p1, p2):
                return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
            length = dist([x1, y1],
                          self.neighbour_sills[(path_start_zone, path[0])])
            if len(path) > 1:
                path.insert(0, path_start_zone)
                for seg1, seg2, seg3 in zip(path[:-2], path[1:-1], path[2:]):
                    length += dist(self.neighbour_sills[(seg1, seg2)],
                                   self.neighbour_sills[(seg2, seg1)])
                    length += dist(self.neighbour_sills[(seg2, seg1)],
                                   self.neighbour_sills[(seg2, seg3)])
                path.pop(0)
                length += dist(self.neighbour_sills[(path[-2], path_final_zone)],
                               self.neighbour_sills[(path_final_zone, path[-2])])
                length += dist([x2, y2],
                               self.neighbour_sills[(path_final_zone, path[-2])])
            else:
                length += dist(self.neighbour_sills[(path_start_zone,
                                                     path_final_zone)],
                               self.neighbour_sills[(path_final_zone,
                                                     path_start_zone)])
                length += dist([x2, y2],
                               self.neighbour_sills[(path_final_zone,
                                                     path_start_zone)])

            return length

        all_paths.sort(key=path_length)
        return all_paths
