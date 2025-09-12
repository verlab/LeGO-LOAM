#include "utility.h"

// Global configuration instance
LidarConfig lidarConfig;

// Legacy external variables for backward compatibility
string pointCloudTopic;
string imuTopic;
string fileDirectory;
bool useCloudRing;
int N_SCAN;
int Horizon_SCAN;
float ang_res_x;
float ang_res_y;
float ang_bottom;
int groundScanInd;
float sensorMountAngle;
float sensorMinimumRange;
float scanPeriod;
int systemDelay;
int imuQueLength;

// Updated segment alpha values (will be computed after config load)
float segmentAlphaX;
float segmentAlphaY;

// Define const variables that were previously defined in utility.h
const bool loopClosureEnableFlag = true;
const double mappingProcessInterval = 0.3;
const float segmentTheta = 60.0/180.0*M_PI; // decrese this value may improve accuracy
const int segmentValidPointNum = 5;
const int segmentValidLineNum = 3;

const int edgeFeatureNum = 2;
const int surfFeatureNum = 4;
const int sectionsTotal = 6;
const float edgeThreshold = 0.1;
const float surfThreshold = 0.1;
const float nearestFeatureSearchSqDist = 25;

// Mapping Params
const float surroundingKeyframeSearchRadius = 50.0; // key frame that is within n meters from current pose will be considerd for scan-to-map optimization (when loop closure disabled)
const int   surroundingKeyframeSearchNum = 50; // submap size (when loop closure enabled)
// history key frames (history submap for loop closure)
const float historyKeyframeSearchRadius = 7.0; // key frame that is within n meters from current pose will be considerd for loop closure
const int   historyKeyframeSearchNum = 25; // 2n+1 number of hostory key frames will be fused into a submap for loop closure
const float historyKeyframeFitnessScore = 0.3; // the smaller the better alignment

const float globalMapVisualizationSearchRadius = 500.0; // key frames with in n meters will be visualized

bool loadLidarConfig(const string& configFile) {
    try {
        YAML::Node config = YAML::LoadFile(configFile);
        
        // Load model information
        lidarConfig.model = config["model"].as<string>();
        
        // Load topic configuration
        lidarConfig.pointCloudTopic = config["topics"]["point_cloud"].as<string>();
        lidarConfig.imuTopic = config["topics"]["imu"].as<string>();
        
        // Load scan configuration
        lidarConfig.N_SCAN = config["scan"]["N_SCAN"].as<int>();
        lidarConfig.Horizon_SCAN = config["scan"]["Horizon_SCAN"].as<int>();
        lidarConfig.ang_res_x = config["scan"]["ang_res_x"].as<float>();
        lidarConfig.ang_res_y = config["scan"]["ang_res_y"].as<float>();
        lidarConfig.ang_bottom = config["scan"]["ang_bottom"].as<float>();
        lidarConfig.groundScanInd = config["scan"]["groundScanInd"].as<int>();
        
        // Load ring channel configuration
        lidarConfig.useCloudRing = config["useCloudRing"].as<bool>();
        
        // Load file output configuration
        lidarConfig.fileDirectory = config["fileDirectory"].as<string>();
        
        // Load sensor mounting configuration
        lidarConfig.sensorMountAngle = config["sensorMountAngle"].as<float>();
        lidarConfig.sensorMinimumRange = config["sensorMinimumRange"].as<float>();
        
        // Load processing parameters
        lidarConfig.scanPeriod = config["scanPeriod"].as<float>();
        lidarConfig.systemDelay = config["systemDelay"].as<int>();
        lidarConfig.imuQueLength = config["imuQueLength"].as<int>();
        
        // Update legacy variables for backward compatibility
        pointCloudTopic = lidarConfig.pointCloudTopic;
        imuTopic = lidarConfig.imuTopic;
        fileDirectory = lidarConfig.fileDirectory;
        useCloudRing = lidarConfig.useCloudRing;
        N_SCAN = lidarConfig.N_SCAN;
        Horizon_SCAN = lidarConfig.Horizon_SCAN;
        ang_res_x = lidarConfig.ang_res_x;
        ang_res_y = lidarConfig.ang_res_y;
        ang_bottom = lidarConfig.ang_bottom;
        groundScanInd = lidarConfig.groundScanInd;
        sensorMountAngle = lidarConfig.sensorMountAngle;
        sensorMinimumRange = lidarConfig.sensorMinimumRange;
        scanPeriod = lidarConfig.scanPeriod;
        systemDelay = lidarConfig.systemDelay;
        imuQueLength = lidarConfig.imuQueLength;
        
        // Compute dependent values
        segmentAlphaX = ang_res_x / 180.0 * M_PI;
        segmentAlphaY = ang_res_y / 180.0 * M_PI;
        
        ROS_INFO("Loaded LiDAR configuration for %s", lidarConfig.model.c_str());
        ROS_INFO("  N_SCAN: %d, Horizon_SCAN: %d", N_SCAN, Horizon_SCAN);
        ROS_INFO("  ang_res_x: %.2f°, ang_res_y: %.2f°", ang_res_x, ang_res_y);
        ROS_INFO("  Topics - Point Cloud: %s, IMU: %s", pointCloudTopic.c_str(), imuTopic.c_str());
        
        return true;
    }
    catch (const YAML::Exception& e) {
        ROS_ERROR("Error loading LiDAR configuration from %s: %s", configFile.c_str(), e.what());
        return false;
    }
    catch (const std::exception& e) {
        ROS_ERROR("Error loading LiDAR configuration: %s", e.what());
        return false;
    }
}
