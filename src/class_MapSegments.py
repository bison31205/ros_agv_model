class Segment:
    x1 = int()
    x2 = int()
    y1 = int()
    y2 = int()

    def __init__(self):
        pass


class MapSegments:
    def __init__(self):
        self.segments = []
        self.segment_value = dict()

    def find_segment(self, x, y):
        for item in self.segments:
            if (((x > item.x1) and (x < item.x2)) and
                    ((y > item.y1) and (y < item.y2))):
                return self.segment_value[item]
        return 0

    def add_segment(self, x1, y1, x2, y2, value):
        temp = Segment()
        [temp.x1, temp.x2] = [x1, x2] if x1 < x2 else [x2, x1]
        [temp.y1, temp.y2] = [y1, y2] if y1 < y2 else [y2, y1]

        self.segments.append(temp)
        self.segment_value[temp] = value
