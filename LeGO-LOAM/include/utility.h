#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_


#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include "cloud_msgs/cloud_info.h"

//#include <pcl/kdtree/kdtree_flann.h>
#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
 
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>
#include <yaml-cpp/yaml.h>

#define PI 3.14159265

using namespace std;

typedef pcl::PointXYZI  PointType;

// LiDAR Configuration Structure
struct LidarConfig {
    // Model information
    string model;
    
    // Topic configuration
    string pointCloudTopic;
    string imuTopic;
    
    // Scan parameters
    int N_SCAN;
    int Horizon_SCAN;
    float ang_res_x;
    float ang_res_y;
    float ang_bottom;
    int groundScanInd;
    
    // Ring channel configuration
    bool useCloudRing;
    
    // File output
    string fileDirectory;
    
    // Sensor mounting
    float sensorMountAngle;
    float sensorMinimumRange;
    
    // Processing parameters
    float scanPeriod;
    int systemDelay;
    int imuQueLength;
};

// Global configuration instance
extern LidarConfig lidarConfig;

// Configuration loading function
bool loadLidarConfig(const string& configFile);

// Legacy external variables for backward compatibility
extern string pointCloudTopic;
extern string imuTopic;
extern string fileDirectory;
extern bool useCloudRing;
extern int N_SCAN;
extern int Horizon_SCAN;
extern float ang_res_x;
extern float ang_res_y;
extern float ang_bottom;
extern int groundScanInd;
extern float sensorMountAngle;
extern float sensorMinimumRange;
extern float scanPeriod;
extern int systemDelay;
extern int imuQueLength;

extern const bool loopClosureEnableFlag;
extern const double mappingProcessInterval;
extern const float segmentTheta; // decrese this value may improve accuracy
extern const int segmentValidPointNum;
extern const int segmentValidLineNum;
extern float segmentAlphaX; // computed after config load
extern float segmentAlphaY; // computed after config load


extern const int edgeFeatureNum;
extern const int surfFeatureNum;
extern const int sectionsTotal;
extern const float edgeThreshold;
extern const float surfThreshold;
extern const float nearestFeatureSearchSqDist;


// Mapping Params
extern const float surroundingKeyframeSearchRadius; // key frame that is within n meters from current pose will be considerd for scan-to-map optimization (when loop closure disabled)
extern const int   surroundingKeyframeSearchNum; // submap size (when loop closure enabled)
// history key frames (history submap for loop closure)
extern const float historyKeyframeSearchRadius; // key frame that is within n meters from current pose will be considerd for loop closure
extern const int   historyKeyframeSearchNum; // 2n+1 number of hostory key frames will be fused into a submap for loop closure
extern const float historyKeyframeFitnessScore; // the smaller the better alignment

extern const float globalMapVisualizationSearchRadius; // key frames with in n meters will be visualized


struct smoothness_t{ 
    float value;
    size_t ind;
};

struct by_value{ 
    bool operator()(smoothness_t const &left, smoothness_t const &right) { 
        return left.value < right.value;
    }
};

/*
    * A point cloud type that has "ring" channel
    */
struct PointXYZIR
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIR,  
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (uint16_t, ring, ring)
)

/*
    * A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp)
    */
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time)
)

typedef PointXYZIRPYT  PointTypePose;

#endif
