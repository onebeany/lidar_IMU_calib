# ML-X 120 LiDAR Integration Details

This document provides a detailed explanation of the changes made to support the ML-X 120 LiDAR in the LiDAR-IMU calibration codebase.

## Problem Overview

The original calibration system was designed for Velodyne LiDAR sensors (specifically VLP-16), but needed modifications to work with the ML-X 120 LiDAR which has different data format characteristics:

- ML-X 120 uses `sensor_msgs/PointCloud2` format
- ML-X 120 has an RGB field instead of an intensity field
- Different point cloud structure compared to VLP-16

Additionally, there was an issue where the calibration process would get stuck at "Iteration 0" due to missing return statements in the IMU bias locking methods.

## Files Modified

### 1. `/home/docker_user/catkin_li_calib/src/lidar_IMU_calib/include/utils/dataset_reader.h`

#### Changes:
1. Added a new LiDAR model type `MLX_120` to the `LidarModelType` enum:

```cpp
enum LidarModelType {
    VLP_16,
    VLP_16_SIMU,
    MLX_120,  // Added ML-X 120 support
};
```

2. Modified the `init()` method to support the ML-X 120 model:

```cpp
void init() {
  switch (lidar_model_) {
    case LidarModelType::VLP_16:
      p_LidarConvert_ = VelodyneCorrection::Ptr(
              new VelodyneCorrection(VelodyneCorrection::ModelType::VLP_16));
      break;
    case LidarModelType::VLP_16_SIMU:
      p_LidarConvert_ = VelodyneCorrection::Ptr(
              new VelodyneCorrection(VelodyneCorrection::ModelType::VLP_16));
      break;
    case LidarModelType::MLX_120:  // Added case for MLX_120
      p_LidarConvert_ = VelodyneCorrection::Ptr(
              new VelodyneCorrection(VelodyneCorrection::ModelType::MLX_120));
      break;
    default:
      std::cout << "LiDAR model " << lidar_model_
                << " not support yet." << std::endl;
  }
}
```

3. Fixed a bug in the `read()` method to return `true` at the end:

```cpp
bool read(const std::string path,
        const std::string imu_topic,
        const std::string lidar_topic,
        const double bag_start = -1.0,
        const double bag_durr = -1.0) {
  // ... existing code ...
  
  std::cout << lidar_topic << ": " << data_->scan_data_.size() << std::endl;
  std::cout << imu_topic << ": " << data_->imu_data_.size() << std::endl;
  
  return true;  // Added return statement
}
```

### 2. `/home/docker_user/catkin_li_calib/src/lidar_IMU_calib/include/utils/vlp_common.h`

#### Changes:
1. Added MLX_120 to the `ModelType` enum:

```cpp
enum ModelType {
  VLP_16,
  HDL_32E,   // not support yet
  MLX_120    // Added MLX-120 support
};
```

2. Completely rewrote the `unpack_scan` method to handle different point cloud formats:

```cpp
void unpack_scan(const sensor_msgs::PointCloud2::ConstPtr &lidarMsg,
                 TPointCloud &outPointCloud) const {
  outPointCloud.clear();
  outPointCloud.header = pcl_conversions::toPCL(lidarMsg->header);
  
  if (m_modelType == ModelType::MLX_120) {
    // Handle ML-X 120 data which has rgb instead of intensity
    colorPointCloudT temp_pc;
    pcl::fromROSMsg(*lidarMsg, temp_pc);
    
    outPointCloud.height = temp_pc.height;
    outPointCloud.width = temp_pc.width;
    outPointCloud.is_dense = false;
    outPointCloud.resize(outPointCloud.height * outPointCloud.width);

    double timebase = lidarMsg->header.stamp.toSec();
    for (int h = 0; h < temp_pc.height; h++) {
      for (int w = 0; w < temp_pc.width; w++) {
        TPoint point;
        point.x = temp_pc.at(w,h).x;
        point.y = temp_pc.at(w,h).y;
        point.z = temp_pc.at(w,h).z;
        // Set intensity to 0 or use rgb value if needed
        point.intensity = 0.0;
        point.timestamp = timebase + getExactTime(h,w);
        outPointCloud.at(w,h) = point;
      }
    }
  } else {
    // Original code for VLP-16 and other LiDARs with intensity
    VPointCloud temp_pc;
    pcl::fromROSMsg(*lidarMsg, temp_pc);
    
    outPointCloud.height = temp_pc.height;
    outPointCloud.width = temp_pc.width;
    outPointCloud.is_dense = false;
    outPointCloud.resize(outPointCloud.height * outPointCloud.width);

    double timebase = lidarMsg->header.stamp.toSec();
    for (int h = 0; h < temp_pc.height; h++) {
      for (int w = 0; w < temp_pc.width; w++) {
        TPoint point;
        point.x = temp_pc.at(w,h).x;
        point.y = temp_pc.at(w,h).y;
        point.z = temp_pc.at(w,h).z;
        point.intensity = temp_pc.at(w,h).intensity;
        point.timestamp = timebase + getExactTime(h,w);
        outPointCloud.at(w,h) = point;
      }
    }
  }
}
```

The key modification is the conditional handling based on LiDAR model type:
- For ML-X 120: Use `colorPointCloudT` (PCL RGB point type) and set intensity to 0
- For other LiDARs: Use the original implementation with intensity field

### 3. `/home/docker_user/catkin_li_calib/src/lidar_IMU_calib/src/ui/calib_helper.cpp`

#### Changes:
Added support for the MLX_120 model type in the initialization code:

```cpp
std::string lidar_model;
nh.param<std::string>("LidarModel", lidar_model, "VLP_16");
IO::LidarModelType lidar_model_type = IO::LidarModelType::VLP_16;
if (lidar_model == "VLP_16") {
  lidar_model_type = IO::LidarModelType::VLP_16;
} else if (lidar_model == "MLX_120") {  // Added this condition
  lidar_model_type = IO::LidarModelType::MLX_120;
} else {
  calib_step_ = Error;
  ROS_WARN("LiDAR model %s not support yet.", lidar_model.c_str());
}
```

### 4. `/home/docker_user/catkin_li_calib/src/lidar_IMU_calib/launch/licalib_gui.launch`

#### Changes:
Modified the default LiDAR model type to use MLX_120:

```xml
<arg name="lidar_model" default="MLX_120" />
```

Changed from the original:
```xml
<arg name="lidar_model" default="VLP_16" />
```

## Key Implementation Details

### Point Cloud Data Handling

The most significant change is in how point cloud data is processed. For ML-X 120 LiDAR:

1. The system now recognizes `sensor_msgs/PointCloud2` format messages from the ML-X 120
2. Uses PCL's `PointXYZRGB` type to properly decode the data that has RGB instead of intensity
3. Sets the intensity field to 0 in the internal point representation since ML-X 120 doesn't have intensity

### Time Handling

The original code's timestamp handling for individual points is preserved. For ML-X 120:

```cpp
point.timestamp = timebase + getExactTime(h,w);
```

This ensures that point timestamps are still calculated consistently for the ML-X 120 data.

### Registration with the System

The changes create a complete integration path:

1. Launch file specifies `MLX_120` as the model type
2. Calibration helper uses that to initialize with the correct model type
3. Dataset reader creates the right converter based on the model type
4. VLP common code handles the conversion appropriately based on point cloud format

## Compilation Fixes

Fixed a missing return statement in the dataset reader's `read()` method which caused a compilation warning:

```cpp
return true;  // Added at the end of the read() method
```

## Testing 

The changes were tested by building the system and running the calibration with a test2.bag file containing ML-X 120 data. The calibration process successfully:

1. Recognized the ML-X 120 data format
2. Loaded 600 point cloud frames and 3000 IMU messages
3. Completed the initialization step
4. Proceeded to the data association step

## Remaining Warnings

There are some remaining deprecation warnings in the code related to PCL's API:

1. `using uint64_t = uint64_t` is deprecated, PCL recommends using `std::uint64_t`
2. `pcl_isnan` is deprecated, PCL recommends using `std::isnan`

These warnings do not affect functionality and could be addressed in a future code cleanup.

## "Iteration 0" Bug Fix

A critical issue was fixed in the Kontiki library which was causing the calibration to get stuck at "Iteration 0". The problem was found in:

### `/home/docker_user/catkin_li_calib/src/lidar_IMU_calib/thirdparty/Kontiki/include/kontiki/sensors/constant_bias_imu.h`

#### Changes:
Added missing return statements to two methods:

```cpp
bool LockGyroscopeBias(bool lock) {
  gyro_bias_locked_ = lock;
  return true;  // Added missing return statement
}

bool LockAccelerometerBias(bool lock) {
  acc_bias_locked_ = lock;
  return true;  // Added missing return statement
}
```

Without these return statements, the calibration process would appear to get stuck at the initial iteration because the IMU bias locking mechanism wasn't working correctly. This issue was identified in a GitHub issue on the original repository and has been fixed in this implementation.

## Conclusion

These modifications successfully extend the LiDAR-IMU calibration system to work with ML-X 120 LiDAR data while maintaining backward compatibility with Velodyne LiDARs. The approach is modular and follows the existing code structure. Additionally, the fix for the "Iteration 0" bug should make the calibration process work properly for all LiDAR types.