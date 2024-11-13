#!/bin/env python3
import csv
import os
from pathlib import Path

import plotly.graph_objs as go


def extract_cpu_usage_from_csv_files():
    dir_to_process = str(os.getcwd())
    cpu_usage_data = {}

    # Find all CSV files starting with 'CPU_'
    for filename in os.listdir(dir_to_process):
        if filename.startswith("CPU_") and (
            filename.endswith(".log") or filename.endswith(".csv")
        ):
            full_path = os.path.join(dir_to_process, filename)
            exp_name = filename[6:].replace(".log", "")
            exp_name = (
                exp_name.replace(".csv", "")
                .replace("node_", "")
                .replace("___", ": ")
                .replace("component_container", "CC")
            )

            with open(full_path, mode="r") as file:
                csv_reader = csv.reader(file)
                next(csv_reader, None)  # Skip header
                cpu_usages = [float(row[1]) for row in csv_reader if len(row) > 1]
                cpu_usage_data[exp_name] = cpu_usages

    return dict(sorted(cpu_usage_data.items()))


def plot_cpu_usage_with_plotly(data):
    fig = go.Figure()

    for file_name, usages in data.items():
        is_ros1 = "1:" in file_name
        if "1:" in file_name:
            marker_shape = "square"
        elif "CYCL" in file_name:
            marker_shape = "x"
        elif "FAST" in file_name:
            marker_shape = "circle"
        else:
            marker_shape = "diamond"
            # Add a line plot of CPU usage
        fig.add_trace(
            go.Scatter(
                y=usages,
                mode="lines+markers",
                marker=dict(
                    symbol=marker_shape,
                    size=7,
                    line=dict(width=1, color="black"),
                ),
                name=file_name,
            )
        )
    fig.update_traces(mode="markers+lines", hovertemplate=None)
    fig.update_layout(hovermode="x")

    fig.update_layout(
        title=f"CPU Usage over Time: {Path(os.getcwd()).stem}",
        xaxis_title="Time (Interval Index)",
        yaxis_title="CPU Usage (%)",
    )

    fig.show()


if __name__ == "__main__":
    # Example usage
    cpu_usage_dict = extract_cpu_usage_from_csv_files()
    plot_cpu_usage_with_plotly(cpu_usage_dict)
