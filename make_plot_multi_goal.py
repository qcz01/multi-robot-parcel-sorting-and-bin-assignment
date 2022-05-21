import os
import sys
import random
import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

FIGSIZE = (16, 9)
MARKERS = list(Line2D.markers.keys())
plt.rcParams.update(
    {
        "font.size": 22,
        "lines.linewidth": 3,
        "lines.markersize": 10,
        "axes.linewidth": 3,
        "xtick.major.width": 3,
        "xtick.minor.width": 3,
        "ytick.major.width": 3,
        "ytick.minor.width": 3,
        "xtick.major.size": 10,
        "xtick.minor.size": 10,
        "ytick.major.size": 10,
        "ytick.minor.size": 10,
    }
)


def make_plot(file_dir: str):
    dir_name = os.path.dirname(file_dir)
    base_name = os.path.splitext(os.path.basename(file_dir))[0]
    # Get data
    with open(file_dir, "r") as csv_file:
        csv_data = pd.read_csv(csv_file)
    x = csv_data["num_robots"].to_numpy()
    data = dict()
    for key in csv_data.keys():
        if "num_robots" not in key and "Unnamed" not in key:
            data[key] = csv_data[key].to_numpy()
    # Plot
    fig = plt.figure(figsize=FIGSIZE)
    ax = fig.add_subplot(111)
    for i, alg_name in enumerate(data):
        dash_length = 5 + random.random() * 10
        ax.plot(
            x,
            data[alg_name],
            label=alg_name,
            linestyle="--",
            dashes=(dash_length, 5),
            marker=MARKERS[i],
        )
    ax.set_xlabel("Number of Robots")
    if base_name.split("-")[3] == "throughput":
        ax.set_ylabel("throughput (higher is better)")
    if base_name.split("-")[3] == "computation_time":
        ax.set_ylabel("total computation time (lower is better)")
    if base_name.split("-")[3] == "computation_time_per_step":
        ax.set_ylabel("computation time per step (lower is better)")
    ax.legend()
    ax.set_title(base_name)
    fig.tight_layout()
    fig.savefig(os.path.join(dir_name, base_name + ".jpg"))


if __name__ == "__main__":
    make_plot(sys.argv[1])
