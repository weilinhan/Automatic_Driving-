#!/usr/bin/env python3
import argparse
import pathlib
import logging
import matplotlib.pyplot as plt
import helpers.Constants as C
import helpers.Functions as helpers
import helpers.Classes as Classes

# maps used for output image formating
COLOR_MAP = {
    Classes.LaneMarkings.CENTER_KEY: "black",
    Classes.LaneMarkings.LEFT_KEY: "black",
    Classes.LaneMarkings.RIGHT_KEY: "black"}

LINE_STYLE_MAP = {
    Classes.LaneMarkings.CENTER_KEY: "--",
    Classes.LaneMarkings.LEFT_KEY: "-",
    Classes.LaneMarkings.RIGHT_KEY: "-"}


def generatePlot(file: pathlib.Path, trajectory: Classes.Trajectory, lanes):
    """Generate a plot and save it to file."""
    _, ax = plt.subplots(ncols=1, nrows=1, dpi=300)

    plt.xlabel("x")
    plt.ylabel("y")

    plt.grid(color='#D3D3D3', linestyle='-', linewidth=1)

    plt.plot(
        trajectory.x,
        trajectory.y,
        marker="o",
        linestyle="-",
        color="red",
        label="trajectory")

    lanesDict = lanes.toPlotFormat()
    for key in lanesDict.keys():
        plt.plot(
            lanesDict[key].x,
            lanesDict[key].y,
            marker="o",
            linestyle=LINE_STYLE_MAP[key],
            color=COLOR_MAP[key])

    plt.savefig(file, bbox_inches="tight")


if __name__ == "__main__":
    # setup log level for all info
    logging.getLogger().setLevel(logging.INFO)

    # setup required arguments
    parser = argparse.ArgumentParser(
        prog="TrajectoryPlotter",
        description="Generate a PNG containg lane markings and the resulting"
        "trajectory. Requires a file containg the lane markings and a file "
        "containg the trajectory. Pairs will be matched by filenames, "
        "if multiple files are specified.",
    )

    parser.add_argument(
        "-l",
        "--lanes",
        help="Single or multiple .lanes file to read markings from.",
        nargs='*',
        action='extend',
        required=True)
    parser.add_argument(
        "-t",
        "--trajectories",
        help="Single or multiple files to read trajectories from.",
        nargs='*',
        action='extend',
        required=True)
    parser.add_argument(
        "-o",
        "--outdir",
        help="Output directory for generated images, default is"
        " input directory.",
    )

    args = parser.parse_args()

    # create path objects for files
    for trajectory in args.trajectories:
        processed = False
        trajPath = pathlib.Path(trajectory)

        # search for matching .lanes file in input
        for lane in args.lanes:
            lanesPath = pathlib.Path(lane)

            # match file if stems and path matches
            if (trajPath.stem == lanesPath.stem
                    and trajPath.parent == lanesPath.parent):

                # try to parse both files
                trajectory = helpers.parseTrajectory(trajPath)
                lanes = helpers.parseLanes(lanesPath)

                # create default path
                outFile = trajPath.parent / (trajPath.stem + C.OUTPUT_FORMAT)
                # overwrite default path if user specified one
                if args.outdir is not None:
                    outFile = pathlib.Path(
                        args.outdir) / (trajPath.stem + C.OUTPUT_FORMAT)
                    # generate plot and write to file

                generatePlot(outFile, trajectory, lanes)
                logging.info(f'Generated output image: "{outFile.absolute()}"')

                # skip other .lanes files
                processed = True
                break

        if not processed:
            logging.warning(
                f"failed to find matching .lanes file for '{trajectory}'")
