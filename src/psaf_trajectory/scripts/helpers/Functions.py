import pathlib
from . import Constants as C
from . import Classes

# default mode used for parsing
MODE_SEARCHING = "searching"

# values for file formatting
NEWLINE = "\n"
TRAJECTORY_PREAMBLE = "trajectory:"
LEFT_LANE_PREAMBLE = "left_lane:"
RIGHT_LANE_PREAMBLE = "right_lane:"
CENTER_LANE_PREAMBLE = "center_lane:"
PREAMBLES = [LEFT_LANE_PREAMBLE, RIGHT_LANE_PREAMBLE, CENTER_LANE_PREAMBLE]


def parseLanes(file: pathlib.Path) -> Classes.LaneMarkings:
    """Read a lanes file from the specified file parameter."""
    if not file.exists():
        raise ValueError(f"Specified lanes file='{file}' does not exist!")

    if file.suffix not in C.LANEFILES:
        raise ValueError(f"Specified lane file='{file}' ending"
                         " does not match accepted='{C.LANEFILES}'")

    mode = ""
    lanes = {
        LEFT_LANE_PREAMBLE: [],
        RIGHT_LANE_PREAMBLE: [],
        CENTER_LANE_PREAMBLE: []
    }

    with file.open() as f:
        for line in f:
            # clean line
            line = line.strip()

            # skip empty line
            if len(line) == 0:
                continue

            # check if line is preamble
            if line in PREAMBLES:
                mode = line
                continue

            # if we are still searching skip point parsing
            if mode not in PREAMBLES:
                continue

            # append point to fitting lane
            lanes[mode].append(Classes.Point.fromLine(line))

    return Classes.LaneMarkings(
        lanes[RIGHT_LANE_PREAMBLE],
        lanes[CENTER_LANE_PREAMBLE],
        lanes[LEFT_LANE_PREAMBLE])


def parseTrajectory(file: pathlib.Path) -> Classes.Trajectory:
    """Read a trajectory from file and return a Trajectory object."""
    if not file.exists():
        raise RuntimeError(
            f"Specified trajectory file='{file}' does not exist!")

    if file.suffix not in C.TRAJFILES:
        raise RuntimeError(
            f"Specified trajectory file='{file}' suffix does not match allowed"
            f"suffixes='{C.TRAJFILES}'!)"
        )

    points = []
    once = True

    with file.open() as f:
        for line in f:
            # clean line
            line = line.rstrip()

            # skip empty line
            if len(line) == 0:
                continue

            # check first line is header
            if once:
                once = False
                if line != TRAJECTORY_PREAMBLE:
                    raise RuntimeError(
                        f"First non-empty line='{line}' didnt"
                        f"match TRAJECTORY_PREAMBLE='{TRAJECTORY_PREAMBLE}'!"
                    )
                # skip header line
                continue

            # start parsing points
            points.append(Classes.Point.fromLine(line))

    return Classes.Trajectory(points)


def writeTrajectory(file: pathlib.Path, points: []):
    """Write a libpsaf_msgs.msg.Trajectory to a file."""
    # check that the file suffix is valid
    if file.suffix not in C.TRAJFILES:
        raise ValueError(f"File ending='{file.suffix}' is not"
                         f" in allowed set='{C.TRAJFILES}'")

    # actually write the file
    with file.open("w") as f:
        # write our preamble
        f.write(f"{TRAJECTORY_PREAMBLE}{NEWLINE}")

        # write each point
        for point in points:
            f.write(f"{point.x}{C.DELIMITER} {point.y}{C.DELIMITER}"
                    f" {point.z}{NEWLINE}")


def writeLaneMarkings(
        file: pathlib.Path,
        left_lane: [],
        right_lane: [],
        center_lane: []):
    """Write a libpsaf_msgs.msg.LaneMarkings to file."""
    # check that file suffx is valid
    if file.suffix not in C.LANEFILES:
        raise ValueError(
            f"File ending='{file.suffix}' is not in"
            f" allowed set='{C.LANEFILES}'")

    pointsMap = {
        LEFT_LANE_PREAMBLE: left_lane,
        RIGHT_LANE_PREAMBLE: right_lane,
        CENTER_LANE_PREAMBLE: center_lane}

    # actually write the traj file
    with file.open("w") as f:
        for key in pointsMap.keys():
            f.write(f"{key}{NEWLINE}")
            for point in pointsMap[key]:
                f.write(
                    f"{point.x}{C.DELIMITER} {point.y}{C.DELIMITER}"
                    f" {point.z}{NEWLINE}")
