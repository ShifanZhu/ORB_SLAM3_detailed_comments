import subprocess
import os
import time

# Fixed arguments
orb_executable = "./Examples/RGB-D/rgbd_cear"
vocab_path = "Vocabulary/ORBvoc.txt"
config_path = "Examples/RGB-D/cear.yaml"

# Dataset directories to process
dataset_dirs = [
    os.path.expanduser("~/data/cear/outdoor/around_building_day_comb"),
    os.path.expanduser("~/data/cear/outdoor/around_building_night_comb"),
    # Add more as needed
]

# Launch control
launch_delay_short = 700  # seconds between launches
processes = []

# Launch each ORB-SLAM3 process
for i, dataset_dir in enumerate(dataset_dirs):
    timestamp_file = os.path.join(dataset_dir, "realsense_timestamp.txt")
    command = [
        orb_executable,
        vocab_path,
        config_path,
        dataset_dir,
        timestamp_file
    ]

    print(f"\n===🚀 Launching ORB-SLAM3 {i + 1}/{len(dataset_dirs)} for {dataset_dir} ===")
    proc = subprocess.Popen(command)
    processes.append((i, proc))  # track by index

    # Monitor already launched
    print("  --- Monitoring running processes ---")
    for j, p in processes:
        ret = p.poll()
        if ret is not None:
            print(f"[✅] Process {j + 1} exited with code {ret}")
        else:
            print(f"[⏳] Process {j + 1} still running")

    time.sleep(launch_delay_short)

# Final wait for all processes
print("\n=== All ORB-SLAM3 processes launched. Waiting for completion. ===")
for i, proc in processes:
    if proc.poll() is None:
        proc.wait()
        print(f"✅ ORB-SLAM3 Process {i + 1} completed.")
