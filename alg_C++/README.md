BARRIER DETECTION – C++ ROS2 NODE

This package implements a LiDAR-based barrier arm detection system. It processes point cloud data from a ROS2 bag file, performs clustering, and classifies barrier arms as OPEN or CLOSED based on their orientation angle. The detection pipeline includes:

1. ROI cropping – Remove ground points and focus on relevant region
2. Statistical outlier removal – Clean noise from the point cloud
3. Voxel downsampling – Reduce point density for performance
4. DBSCAN clustering – Group points into candidate objects
5. PCA feature extraction – Compute length, width, and orientation
6. Best cluster selection – Score clusters by thinness and length
7. Classification – Determine OPEN/CLOSED based on angle threshold

TABLE OF CONTENTS:
1. DEPENDENCIES
2. BUILDING THE PROJECT
3. UPDATE BAG FILE PATH
4. EXECUTE
5. ALGORITHM DESCRIPTION
6. KEY IMPLEMENTATION NOTES

### 1. DEPENDENCIES ###
============================================================
 Package         | Version 
 Ubuntu          | 22.04
 ROS2            | Humble 
 PCL             | 1.10+  
 Eigen3          | 3.3+  
 rosbag2_cpp     | -    
 pcl_conversions | -    

```bash
# ROS2 Humble (if not already installed)
sudo apt install ros-humble-rosbag2-storage-default-plugins

# PCL and Eigen
sudo apt install libpcl-dev libeigen3-dev

# Build tools
sudo apt install cmake build-essential
============================================================

### 2. BUILDING THE PROJECT ###
============================================================
alg_C++ folder contains:

package.xml
CMakeLists.txt
include
	barrier_detection
		clustering.hpp
		geometry.hpp
		preprocess.hpp
		reader.hpp
src
	algoritm_node.cpp
	clustering.cpp
	geometry.cpp
	preprocess.cpp
	reader.cpp

```bash
1. Source your ROS2 environment
source /opt/ros/humble/setup.bash

2. Create a workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

3. Copy the package into src/
cp -r /path/to/alg_C++ /ros2_ws/src/ 

4. Build
cd /ros2_ws
colcon build --packages-select barrier_detection ### here the package name is barrier_detection, not alg_C++ please don't confuse

5. Source the workspace
source install/setup.bash

6. Verify installation
ros2 pkg list | grep barrier_detection
ros2 pkg executables barrier_detection
============================================================

### 3. UPDATE BAG FILE PATH ###
============================================================
Before running, update the bag file path in `src/algorithm_node.cpp`:
// Find this line in main() function

const std::string bag_path = "/root/Desktop/rosbag2_lidar_data"; ### locate this data file on your own computer
============================================================

## 4. EXECUTE ##
============================================================
```bash
# Source environment (if not already)
source /opt/ros/humble/setup.bash  ### or can be "source /opt/ros/humble/install/setup.bash"
source /ros2_ws/install/setup.bash

# Run the algorithm itself
ros2 run barrier_detection algorithm_node

# At the same time open another terminal window (connect to docker container if the simulation is run on it sudo docker exec -it ID /bin/bash) and open RViz
rviz2
 
# Type into "Fixed Frame" -> lidar_frame3. Data and the simulation will automatically appear on your screen 
# Add displays by topic in the rviz display menu

# ROS TOPICS FOR YOUR RVIZ2 #
| Topic | Type 
|-------|------
| `/lidar_raw` | `sensor_msgs/PointCloud2`
| `/lidar_filtered` | `sensor_msgs/PointCloud2` 
| `/lidar_clusters` | `visualization_msgs/MarkerArray`
| `/barrier_direction` | `visualization_msgs/Marker` 
| `/barrier_state` | `std_msgs/String` | `"OPEN"`, `"CLOSED"`, or `"NO_BARRIER_ARM"` |
| `/barrier_angle` | `std_msgs/Float32` | Angle (degrees) relative to Y-axis |
============================================================

### 5. ALGORITHM DESCRIPTION ###
============================================================
# 1. Preprocessing (`crop_roi`)

| Step            | Parameters    
|------           |------------                       
| ROI crop        | `x=[0,10]`, `y=[-5,5]`, `z=[-1,5]` 
| Outlier removal | `k=8`, `z_thresh=2.5` 
| Voxel grid      | `0.05 m` 

# 2. Clustering (`cluster_dbscan`)

- Algorithm: DBSCAN (density-based)
- `eps`: 0.16 m – neighbourhood radius
- `min_samples`: 20 – minimum points for core point
- Data structure: KD-tree for O(n log n) neighbour queries

# 3. Feature Extraction (`fit_line_pca`)

```cpp
struct PCAResult {
    Eigen::Vector3f direction;   // Principal axis (unit vector)
    Eigen::Vector3f direction2;  // Second axis (for width)
    Eigen::Vector3f centroid;    // Cluster centre
    Eigen::Vector3f eigenvalues; // [λ0, λ1, λ2] descending
    float linearity;             // λ0 / (λ1 + ε)
};
```

Length = projection range on principal axis  
Width = projection range on second axis  
Height = Z-axis range  

# 4. Best Cluster Selection

Score function:

```
score = length / width + 0.5 × length
```

Hard constraints:
- `width > 0.5 m` → reject (too thick for a barrier arm)
- `lw_ratio < 1.0` → reject (not longer than wide)
- `size < 10 points` → reject (insufficient data)

# 5. Classification

```cpp
state = (angle >= 80°) ? "OPEN" : "CLOSED"
```

Angle is computed between the principal axis and the Y-axis `[0, 1, 0]`, clamped to `[0°, 90°]`.

## Configuration Parameters

All parameters are hardcoded with sensible defaults. Modify them directly in the source files:

| Parameter | File | Default | Description |
|-----------|------|---------|-------------|
| `eps` | `clustering.cpp` | 0.16 m | DBSCAN neighbourhood radius |
| `min_samples` | `clustering.cpp` | 20 | DBSCAN minimum core points |
| `voxel_size` | `preprocess.hpp` | 0.05 m | Downsampling leaf size |
| `x_min/max` | `preprocess.hpp` | 0.0 / 10.0 | ROI X bounds (forward) |
| `y_min/max` | `preprocess.hpp` | -5.0 / 5.0 | ROI Y bounds (lateral) |
| `z_min/max` | `preprocess.hpp` | -1.0 / 5.0 | ROI Z bounds (height) |
| `angle_threshold` | `algorithm_node.cpp` | 80° | OPEN/CLOSED threshold |
| `timer_period` | `algorithm_node.cpp` | 100 ms | Processing rate (10 Hz) |
============================================================

### 6. KEY IMPLEMENTATION NOTES ###
============================================================
**PCL 1.10 uses `boost::shared_ptr`** – All cloud pointers must be `boost::shared_ptr`, not `std::shared_ptr`:

---

## Important Notes

1. Bag file path: Update the hardcoded path in `algorithm_node.cpp` before running.

2. Frame rate: The node publishes at 10 Hz (100 ms timer), matching the Python version.

3. Coordinate system: Assumes LiDAR is forward-facing (X forward, Y left, Z up). The reference axis for angle computation is `[0, 1, 0]` (Y-axis).

4. Noise handling: Points labelled as noise (DBSCAN label = -1) are discarded.

5. Empty clusters: If no suitable cluster is found, the node publishes an empty MarkerArray and state = `"NO_BARRIER_ARM"`.

6. Docker environment: If running in Docker, you may need to run `xhost +` before launching RViz2 for display access.
============================================================

## License

Apache 2.0 – see `package.xml` for details.
