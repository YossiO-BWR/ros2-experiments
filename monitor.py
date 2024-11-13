#!/bin/env python3
import psutil
import time
import argparse
import datetime
import os


def get_process_cpu_usage(process_name):
    """Function to get CPU usage of a specific process by name."""
    for process in psutil.process_iter(["name", "cpu_percent"]):
        if process.info["name"] == process_name:
            return process.info["cpu_percent"]
    return None


def monitor_and_log_cpu_usage(
    process_name, notes="", duration=os.getenv("REC_DUR", 60), interval=1
):
    """Monitor and log CPU usage of a process every second for a given duration."""
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    try:
        duration = int(duration)
    except:
        duration = 60
    log_filename = f"CPU___{process_name}___{notes}.csv"
    notes = notes.replace(" ", "_")
    with open(log_filename, "w") as log_file:
        log_file.write(f"ts,cpu_usage\n")
        for i in range(duration):
            ts = time.time_ns()
            cpu_usage = get_process_cpu_usage(process_name)

            if cpu_usage is not None:
                log_entry = f"{i:3}/{duration}  CPU Usage for process '{process_name}': {cpu_usage}%"
            else:
                log_entry = f"{i:3}/{duration}  Process '{process_name}' not found."

            print(log_entry)
            log_file.write(f"{ts},{cpu_usage}\n")

            time.sleep(interval)

    print(f"Logging complete. Results saved to {log_filename}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Monitor and log CPU usage of a specific process."
    )
    parser.add_argument("process_name", type=str, help="Name of the process to monitor")
    parser.add_argument(
        "notes", type=str, help="additional notes to append to file name"
    )
    args = parser.parse_args()

    monitor_and_log_cpu_usage(args.process_name, args.notes)
