![CI](https://github.com/verlab/LeGO-LOAM/actions/workflows/workflow.yml/badge.svg)
# LeGO-LOAM

This repository contains code for a lightweight and ground optimized lidar odometry and mapping (LeGO-LOAM) system for ROS compatible UGVs. The system takes in point cloud  from a Velodyne VLP-16 Lidar (palced horizontally) and optional IMU data as inputs. It outputs 6D pose estimation in real-time. A demonstration of the system can be found here -> https://www.youtube.com/watch?v=O3tz_ftHV48
<!--
[![Watch the video](/LeGO-LOAM/launch/demo.gif)](https://www.youtube.com/watch?v=O3tz_ftHV48)
-->
<p align='center'>
    <img src="/LeGO-LOAM/launch/demo.gif" alt="drawing" width="800"/>
</p>

## Lidar-inertial Odometry

An updated lidar-initial odometry package, [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM), has been open-sourced and available for testing.

## Dependencies

- [ROS](http://wiki.ros.org/ROS/Installation) (tested with indigo, kinetic, and melodic)
- [gtsam](https://github.com/borglab/gtsam/releases) (Georgia Tech Smoothing and Mapping library, 4.0.0-alpha2)
  ```bash
  wget -O ~/Downloads/gtsam.zip https://github.com/borglab/gtsam/archive/4.0.0-alpha2.zip
  cd ~/Downloads/ && unzip gtsam.zip -d ~/Downloads/
  cd ~/Downloads/gtsam-4.0.0-alpha2/
  mkdir build && cd build
  cmake ..
  sudo make install
  ```
- [yaml-cpp](https://github.com/jbeder/yaml-cpp) (YAML configuration file parsing)
  ```bash
  sudo apt-get install libyaml-cpp-dev
  ```

## Compile

You can use the following commands to download and compile the package.

```
cd ~/catkin_ws/src
git clone https://github.com/RobustFieldAutonomyLab/LeGO-LOAM.git
cd ..
catkin_make -j1
```
When you compile the code for the first time, you need to add "-j1" behind "catkin_make" for generating some message types. "-j1" is not needed for future compiling.

## The system

LeGO-LOAM is speficifally optimized for a horizontally placed VLP-16 on a ground vehicle. It assumes there is always a ground plane in the scan. The UGV we are using is Clearpath Jackal. It has a built-in IMU. 

<p align='center'>
    <img src="/LeGO-LOAM/launch/jackal-label.jpg" alt="drawing" width="400"/>
</p>

The package performs segmentation before feature extraction.

<p align='center'>
    <img src="/LeGO-LOAM/launch/seg-total.jpg" alt="drawing" width="400"/>
</p>

Lidar odometry performs two-step Levenberg Marquardt optimization to get 6D transformation.

<p align='center'>
    <img src="/LeGO-LOAM/launch/odometry.jpg" alt="drawing" width="400"/>
</p>

## LiDAR Configuration System

This repository now uses a YAML-based configuration system for different LiDAR sensors, following ROS best practices. Instead of hardcoded constants, sensor parameters are defined in configuration files.

### Supported LiDAR Sensors

The following LiDAR configurations are included:

- **VLP-16**: `config/vlp16.yaml` - Velodyne VLP-16 Puck
- **LSLidar C32W**: `config/lslidar_c32w.yaml` - LSLidar 32-channel (default)
- **HDL-32E**: `config/hdl32e.yaml` - Velodyne HDL-32E
- **VLS-128**: `config/vls128.yaml` - Velodyne VLS-128
- **Ouster OS1-16**: `config/ouster_os1_16.yaml` - Ouster OS1-16
- **Ouster OS1-64**: `config/ouster_os1_64.yaml` - Ouster OS1-64

### Configuration File Structure

Each configuration file contains the following parameters:

```yaml
# LiDAR model name
model: "VLP-16"

# Point cloud and IMU topic configuration
topics:
  point_cloud: "/velodyne_points"
  imu: "/imu/data"

# Scan configuration
scan:
  N_SCAN: 16                    # Number of scan lines
  Horizon_SCAN: 1800           # Number of points per scan line
  ang_res_x: 0.2               # Horizontal angular resolution (degrees)
  ang_res_y: 2.0               # Vertical angular resolution (degrees) 
  ang_bottom: 15.1             # Bottom beam angle offset (degrees)
  groundScanInd: 7             # Ground scan index for ground detection

# Ring channel configuration
useCloudRing: false            # Use ring channel for projection if available

# Other parameters...
```

### Adding New LiDAR Sensors

To add support for a new LiDAR sensor:

1. Create a new YAML configuration file in the `config/` directory
2. Define all required parameters following the existing structure
3. Ensure the point cloud can be properly projected to a range image
4. Verify that ground detection works with sufficient points

**Important**: When implementing a new sensor, make sure that the `ground_cloud` has enough points for matching. The key is ensuring proper range image projection and ground detection.

### Ring Channel Support

The **useCloudRing** flag helps with point cloud projection for sensors with ring channels:
- Velodyne sensors: Use the "ring" channel
- Ouster sensors: Use the "ring" or "r" channel  
- Other sensors: May have similar ring/row identification channels

If using a non-Velodyne LiDAR with a ring channel, you may need to modify the `PointXYZIR` definition in `utility.h` and corresponding code in `imageProjection.cpp`.

### IMU Integration

If using your LiDAR with an IMU:
- Ensure proper alignment between LiDAR and IMU
- The algorithm uses IMU data to correct point cloud distortion from sensor motion
- Misaligned IMU data will deteriorate results
- **Note**: Ouster LiDAR IMU is not fully supported (LeGO-LOAM requires 9-DOF IMU)

## Run the package

### 1. Basic Usage

Run with default configuration (LSLidar C32W):
```bash
roslaunch lego_loam run.launch
```

### 2. Select LiDAR Configuration

Run with a specific LiDAR configuration:
```bash
# For VLP-16
roslaunch lego_loam run.launch lidar_config:=vlp16

# For HDL-32E
roslaunch lego_loam run.launch lidar_config:=hdl32e

# For Ouster OS1-16
roslaunch lego_loam run.launch lidar_config:=ouster_os1_16
```

### 3. Custom Configuration File

Use a custom configuration file:
```bash
roslaunch lego_loam run.launch config_file:=/path/to/your/config.yaml
```

### 4. Example Usage with Bag Files

Play existing bag files with appropriate LiDAR configuration:
```bash
# For VLP-16 data
roslaunch lego_loam run.launch lidar_config:=vlp16
rosbag play *.bag --clock --topic /velodyne_points /imu/data

# For LSLidar data  
roslaunch lego_loam run.launch lidar_config:=lslidar_c32w
rosbag play *.bag --clock --topic /mid/points /imu/imu_and_mag
```

**Notes**: 
- The parameter `/use_sim_time` is set to "true" for simulation, "false" for real robot usage
- IMU data is optional but greatly improves estimation accuracy when provided
- Sample bags can be downloaded from [here](https://github.com/RobustFieldAutonomyLab/jackal_dataset_20170608)
- Topic names are automatically configured based on the selected LiDAR configuration 

## New data-set

This dataset, [Stevens data-set](https://github.com/TixiaoShan/Stevens-VLP16-Dataset), is captured using a Velodyne VLP-16, which is mounted on an UGV - Clearpath Jackal, on Stevens Institute of Technology campus. The VLP-16 rotation rate is set to 10Hz. This data-set features over 20K scans and many loop-closures. 

<p align='center'>
    <img src="/LeGO-LOAM/launch/dataset-demo.gif" alt="drawing" width="600"/>
</p>
<p align='center'>
    <img src="/LeGO-LOAM/launch/google-earth.png" alt="drawing" width="600"/>  
</p>

## Cite *LeGO-LOAM*

Thank you for citing [our *LeGO-LOAM* paper](./Shan_Englot_IROS_2018_Preprint.pdf) if you use any of this code: 
```
@inproceedings{legoloam2018,
  title={LeGO-LOAM: Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain},
  author={Shan, Tixiao and Englot, Brendan},
  booktitle={IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={4758-4765},
  year={2018},
  organization={IEEE}
}
```

## Loop Closure

The loop-closure method implemented in this package is a naive ICP-based method. It often fails when the odometry drift is too large. For more advanced loop-closure methods, there is a package called [SC-LeGO-LOAM](https://github.com/irapkaist/SC-LeGO-LOAM), which features utilizing point cloud descriptor.

## Speed Optimization

An optimized version of LeGO-LOAM can be found [here](https://github.com/facontidavide/LeGO-LOAM/tree/speed_optimization). All credits go to @facontidavide. Improvements in this directory include but not limited to:

    + To improve the quality of the code, making it more readable, consistent and easier to understand and modify.
    + To remove hard-coded values and use proper configuration files to describe the hardware.
    + To improve performance, in terms of amount of CPU used to calculate the same result.
    + To convert a multi-process application into a single-process / multi-threading one; this makes the algorithm more deterministic and slightly faster.
    + To make it easier and faster to work with rosbags: processing a rosbag should be done at maximum speed allowed by the CPU and in a deterministic way.
    + As a consequence of the previous point, creating unit and regression tests will be easier.

