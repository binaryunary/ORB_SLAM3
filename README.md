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

# Running

# Building
I have created a VSCode development container, which can be found at https://github.com/binaryunary/orbslam-dev.
It includes all the dependencies that are required to build ORB-SLAM 3. This is essentially just a Docker image, can
be used without VSCode.

To build
```bash
# Clone the repo somewhere

cd ORB_SLAM3

```

