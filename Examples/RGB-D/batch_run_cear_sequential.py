import subprocess
import os

# Fixed arguments
orb_executable = "./Examples/RGB-D/rgbd_cear"
vocab_path = "Vocabulary/ORBvoc.txt"
config_path = "Examples/RGB-D/cear.yaml"

# List of dataset directories
dataset_dirs = [
  # "~/data/cear/indoor/lab1_well-lit_trot",
  # "~/data/cear/indoor/lab1_well-lit_comb",
  "~/data/cear/indoor/dininghall_well-lit_comb",
  # Add more as needed
]

# Expand ~ to home directory
dataset_dirs = [os.path.expanduser(d) for d in dataset_dirs]

# Run the command for each dataset
for dataset_dir in dataset_dirs:
  timestamp_file = os.path.join(dataset_dir, "realsense_timestamp.txt")
  command = [
      orb_executable,
      vocab_path,
      config_path,
      dataset_dir,
      timestamp_file
  ]
  print(f"Running ORB-SLAM3 for: {dataset_dir}")
  subprocess.run(command)
