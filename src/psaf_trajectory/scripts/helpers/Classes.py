from . import Constants as C


class Point:
    """Simple class to encapsulate a point."""

    def __init__(self, x: float, y: float, z: float):
        self.x = x
        self.y = y
        self.z = z

    def fromLine(line: str):
        split = line.split(C.DELIMITER)
        if len(split) != 3:
            raise RuntimeError(
                "Failed to parse 3 coordinates"
                f" from line='{line}', got='{split}'")
        return Point(float(split[0]), float(split[1]), float(split[2]))


class Trajectory:
    """Simple trajectory container."""

    def __init__(self, points: [Point]):
        self.points = points

        # member lists for plotting access
        self.x = []
        self.y = []
        self.z = []

        for point in self.points:
            self.x.append(point.x)
            self.y.append(point.y)
            self.z.append(point.z)


class LaneMarkings:
    """Simple container for lane markings."""

    LEFT_KEY = "LEFT"
    RIGHT_KEY = "RIGHT"
    CENTER_KEY = "CENTER"

    def __init__(
            self,
            right_lane: [Point],
            center_lane: [Point],
            left_lane: [Point]):
        # we just reuse the trajectory container
        self.right_lane = Trajectory(right_lane)
        self.center_lane = Trajectory(center_lane)
        self.left_lane = Trajectory(left_lane)

    def toPlotFormat(self) -> set:
        # we could just return these three in a certain order
        # but using a dict with keys makes handling these further
        # easier
        return {
            self.LEFT_KEY: self.left_lane,
            self.RIGHT_KEY: self.right_lane,
            self.CENTER_KEY: self.center_lane
        }
