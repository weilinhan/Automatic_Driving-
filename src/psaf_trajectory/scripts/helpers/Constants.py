# file endings used for checking and generating files
OUTFILE_ENDING = ".out"
OUTPUT_FORMAT = ".png"
LANEFILE_ENDING = ".lanes"
LANETRANSFILE_ENDING = ".lanes_transformed"
TRAJECTORYFILE_ENDING = ".traj"
LANEFILES = {LANEFILE_ENDING, LANETRANSFILE_ENDING}
TRAJFILES = {OUTFILE_ENDING, TRAJECTORYFILE_ENDING}

# topic strings
TRAJ_TOPIC = "/trajectory/trajectory"
LANES_TOPIC = "/lane_detection/lane_markings"
LANES_TRANS_TOPIC = "/trajectory/lane_markings_transformed"

# constant values for arguments
TRAJECTORY_FLAG = "trajectory"
LANES_FLAG = "lane_markings"
LANES_TRANS_FLAG = "lane_markings_transformed"

# mapping of file endings to file flags
SUFFIX_MAP = {
    TRAJECTORY_FLAG: TRAJECTORYFILE_ENDING,
    LANES_FLAG: LANEFILE_ENDING,
    LANES_TRANS_FLAG: LANETRANSFILE_ENDING
}

# delimiter for point parsing
DELIMITER = ","
