#!/usr/bin/env python3
import argparse
import pathlib
import logging
import os
import datetime
import rclpy
import rclpy.node
import libpsaf_msgs.msg
import helpers.Constants as C
import helpers.Functions as helpers

TOPICS_DEFAULT_ARG = [C.TRAJECTORY_FLAG, C.LANES_FLAG, C.LANES_TRANS_FLAG]


class TrajectoryLogger(rclpy.node.Node):
    # map for counting filename mode
    countMap = {
        C.TRAJECTORY_FLAG: 0,
        C.LANES_FLAG: 0,
        C.LANES_TRANS_FLAG: 0
    }

    def __init__(
            self,
            outPath: pathlib.Path,
            topics: [] = None,
            timestamp: bool = False):
        super().__init__('TrajectoryLogger')
        self.outPath = outPath
        self.timestamp = timestamp

        for topic in topics:
            if topic is C.TRAJECTORY_FLAG:
                logging.info(f"Subscribing to '{C.TRAJ_TOPIC}'")
                self.create_subscription(
                    libpsaf_msgs.msg.Trajectory,
                    C.TRAJ_TOPIC,
                    self.trajCallback,
                    10)

            elif topic is C.LANES_FLAG:
                logging.info(f"Subscribing to '{C.LANES_TOPIC}'")

                self.create_subscription(
                    libpsaf_msgs.msg.LaneMarkings,
                    C.LANES_TOPIC,
                    self.lanesCallback,
                    10)

            elif topic is C.LANES_TRANS_FLAG:
                logging.info(f"Subscribing to '{C.LANES_TRANS_TOPIC}'")

                self.create_subscription(
                    libpsaf_msgs.msg.LaneMarkings,
                    C.LANES_TRANS_TOPIC,
                    self.lanesTransCallback,
                    10)

            else:
                raise ValueError(f"Unknown topic='{topic}' received")

    # callbacks for different kinds,
    # would be nicer if we could specify callback args
    # then we wouldnt need three functions
    def trajCallback(self, msg):
        self.logFile(C.TRAJECTORY_FLAG, msg)

    def lanesCallback(self, msg):
        self.logFile(C.LANES_FLAG, msg)

    def lanesTransCallback(self, msg):
        self.logFile(C.LANES_TRANS_FLAG, msg)

    def getCount(self, type) -> int:
        if type not in TOPICS_DEFAULT_ARG:
            raise ValueError(f"Invalid argument='{type}', allowed"
                             f" types are='{TOPICS_DEFAULT_ARG}'")
        retVal = self.countMap[type]
        self.countMap[type] += 1
        return retVal

    # method creates output file and passes data along to
    # correct write function
    def logFile(self, type: str, data: str):
        # check that paramter is valid
        if type not in TOPICS_DEFAULT_ARG:
            raise ValueError(f"Invalid argument='{type}', allowed"
                             f" types are='{TOPICS_DEFAULT_ARG}'")

        stem = datetime.datetime.now().isoformat(
        ) if self.timestamp else str(self.getCount(type))

        # get the output file
        outFile = self.outPath / (stem + C.SUFFIX_MAP[type])
        logging.info(f"Got {type}, writing to '{outFile}'")

        # skip if outfile exists
        if outFile.exists():
            logging.warn(f"Log file '{outFile}' already exists, skipping")
            return

        # call correct write function for data
        if type is C.TRAJECTORY_FLAG:
            helpers.writeTrajectory(outFile, data.points)
        elif type is C.LANES_FLAG or type is C.LANES_TRANS_FLAG:
            helpers.writeLaneMarkings(
                outFile,
                data.left_lane,
                data.right_lane,
                data.center_lane)
        else:
            raise ValueError("Invalid argument='{type}', must be in"
                             f" {TOPICS_DEFAULT_ARG}")


if __name__ == "__main__":
    # setup log level for all info
    logging.getLogger().setLevel(logging.INFO)

    # setup required arguments
    parser = argparse.ArgumentParser(
        prog=f"{pathlib.Path(__file__).name}",
        description=f"ROS Node that continously dumps '{C.TRAJ_TOPIC}'"
        f" , '{C.LANES_TOPIC}' and '{C.LANES_TRANS_TOPIC}'."
    )

    parser.add_argument(
        "-o",
        "--outdir",
        help="Output directory for generated images, default is"
        " subfolder 'dump' in input directory.",
        default=pathlib.Path(os.getcwd()) / "dump"
    )

    parser.add_argument(
        "-q",
        "--quiet",
        help="Only log errors.",
        action="store_true",
        default=False
    )

    parser.add_argument(
        "-s",
        "--timestamp",
        help="Change filename from incrementing count to timestamp.",
        action="store_true",
        default=False
    )

    parser.add_argument(
        "-t",
        "--topics",
        help=f"Indicate which topics to dump, default={TOPICS_DEFAULT_ARG}",
        default=TOPICS_DEFAULT_ARG,
        nargs="*",
    )

    args = parser.parse_args()

    if args.quiet:
        logging.getLogger().setLevel(logging.ERROR)

    dumpPath = pathlib.Path(args.outdir)
    if not dumpPath.exists():
        logging.info(f"Output directory '{dumpPath}' does"
                     " not exist, creating it...")
        dumpPath.mkdir(parents=True)

    # check for valid args
    for topic in args.topics:
        if topic not in TOPICS_DEFAULT_ARG:
            raise ValueError(f"Unknown topic='{topic}' received,"
                             f" allowed values are '{TOPICS_DEFAULT_ARG}'")

    # create listener node
    rclpy.init()
    sub = TrajectoryLogger(dumpPath, args.topics, args.timestamp)

    # run listener node until interrupted
    try:
        logging.info("Starting subscriber node")
        rclpy.spin(sub)
    except KeyboardInterrupt:
        pass

    # manually cleanup node
    sub.destroy_node()

    # try to shutdown rclpy, raises error most of the time
    # because it shuts down itself already
    try:
        rclpy.shutdown()
    except Exception:
        pass

    logging.info("Exitting...")
