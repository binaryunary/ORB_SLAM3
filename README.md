# Introduction

This repo is part of my master's thesis.

TODO: Add link to thesis itself.

This is a slightly modified version of of the original ORB-SLAM 3 algorithm that can be found at https://github.com/UZ-SLAMLab/ORB_SLAM3.

Main changes compared to the original:
- GPS coordinates are stored in `Frame` and `KeyFrame`.
- Changed output format.
- SLAM/localization mode switch via CLI.
- Uses Catkin for better building experience.
- New ROS node that passes GPS information to ORB-SLAM 3.

# Running the ROS node
You can run the ROS node either manually

```bash
rosrun orbslam mono_gps <path_to_vocabulary> <path_to_settings> <'mapping'|'localization'> " "<out_dir> [<active_map>]

# Params:
* <path_to_vocabulary>: path to the ORBvoc.txt file
* <path_to_settings>: path to ORB-SLAM 3 settings yaml file
* <'mapping'|'localization'>: operating mode, either mapping(SLAM) or localization only (requires an existing map created in mapping mode)
* <out_dir>: path to output directory where the result files are stored, will be created if it does not exist
* [<active_map>]: (optional) index of the map from Atlas to load (in case it contains multiple maps), defaults to 0
```

or via a supplied roslaunch file
```bash
cd testbench
roslaunch sekonix-mono-gps.launch rosbag_args:='<path_to_bag>' orbslam_mode:=<'mapping'|'localization'> out_dir:=<out_dir>
```

**NOTE**
The GPS data topic is currently hard-coded to `/novatel/oem7/inspva`, it assumes the messages are of type `INSPVA`.
The image topic is hard-coded to `/camera/image_raw`.
See the included roslaunch files to see examples of republishing different camera topics to `/camera/image_raw`.

## Examples

Run in mapping mode to create the Atlas map from a bag:
```bash
# Add --sigint-timeout just in case to give enough time for ORB-SLAM to wrap up, serialize Atlas and write it to disk.
roslaunch --sigint-timeout=120 sekonix-mono-gps.launch rosbag_args:='/path/to.bag' orbslam_mode:=mapping out_dir:=/path/to/results
```

Run in localization mode to localize against the map created in the previous step
```bash
roslaunch sekonix-mono-gps.launch rosbag_args:='/path/to.bag' orbslam_mode:=localization out_dir:=/path/to/results
```

## Output
At the end of a run ORB-SLAM will create several output files.
All files except `atlas.osa` are in the the TUM format (https://cvg.cit.tum.de/data/datasets/rgbd-dataset/file_formats).

### Mapping Mode
- `atlas.osa`: Binary serialization of Atlas, this includes all maps, key frames and map points. This file is loaded when ORB-SLAM is used in localization mode.
- `m_*_trajectory.txt`: Trajectory during mapping in SLAM coordinates, generated from the KeyFrames in the map and used for training the coordinate transformation model.
- `m_*_trajectory_gt_wgs.txt`: Ground truth in WGS84 coordinates, used for visual- ization and for training the coordinate transformation model.
- `m_*_map_points.txt`: x, y, z coordinates of map points, used for visualization.

In the above file names, `*` represents the map index in Atlas. If ORB-SLAM 3 loses tracking and cannot relocalize, it will create a new map and build on that.
If it cannot merge with a previous map, it will remain a disjoint map in Atlas. For training, we pick the longest map (most keyframes) and discard the rest.

### Localization Mode
- `l_*_trajectory.txt`: Trajectory during localization in SLAM coordinates, generated
from all tracked Frames and used as input for the coordinate transformation model.
- `l_*_trajectory_gt_wgs.txt`: Ground truth in WGS84 coordinates, used for visual-
ization.

In this case, `*` indicates the sequence number of localization, and this number increments with each new localization run.
It is worth noting that localization files with index 0 (`l_0_trajectory.txt` and `l_0_trajectory_gt_wgs.txt`) have a special meaning in our system. They are created by running ORB-SLAM 3 in localization mode on the same input data that was used for mapping ("self-localization).

# Building
I have created a VSCode development container, which can be found at https://github.com/binaryunary/orbslam-dev.
It includes all the dependencies that are required to build ORB-SLAM 3. This is essentially just a Docker image, can
be used without VSCode.

To build
```bash
# 0. Clone the repo somewhere

# 1. Build the included third party libs
cd src/orbslam3
./build_thirdparty.sh

# 2. Unpack the ORB vocabulary
cd src/Vocabulary
tar -xvzf ORBvoc.txt.tar.gz

# 3. Build ORB-SLAM 3
catkin build --profile release # This will use a predefined config from .catkin_tools/profiles/release/config.yaml

# 4. Don't forget to source the setup file if you want to use the built ROS node
source install_release/setup.sh

```

