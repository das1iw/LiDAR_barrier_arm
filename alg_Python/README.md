BARRIER DETECTION – Python ROS2 NODE

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
 Python          | 3.8+
 scikit-learn    | (DBSCAN)
 numpy           | 
 scipy           | (cKDTree for outlier removal)
 sensor_msgs_py  |
 rosbag2_py      |

```bash
# ROS2 Humble (if not already installed)
sudo apt install ros-humble-rosbag2-storage-default-plugins

# numpy and scikit-learn
pip install numpy scikit-learn scipy


### 2. FOLDER ###
============================================================
alg_Python folder contains:

algorithm_node.py
clustering.py
geometry.py
preprocess.py
reader.py

============================================================

### 3. UPDATE BAG FILE PATH ###
============================================================
Before running, update the bag file path in `algorithm_node.py`:
// Find this line in main() function

bag_path = "/root/Desktop/rosbag2_lidar_data"; ### locate this data file on your own computer
============================================================

## 4. EXECUTE ##
============================================================
```bash
# Source environment (if not already)
source /opt/ros/humble/setup.bash  ### or can be "source /opt/ros/humble/install/setup.bash"
source /ros2_ws/install/setup.bash

# Run the algorithm itself
python3 algorithm_node.py

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

# 2. Clustering (`cluster()`)

- Algorithm: DBSCAN (density-based)
- `eps`: 0.16 m – neighbourhood radius
- `min_samples`: 20 – minimum points for core point
- sklearn.cluster.DBSCAN

# 3. Feature Extraction (`fit_line_pca`)

Length = projection range on principal axis  
Width = projection range on second axis  
Height = Z-axis range  

# 4. Best Cluster Selection

score = length / width + 0.5 × length

Hard constraints:
- `width > 0.5 m` → reject (too thick for a barrier arm)
- `lw_ratio < 1.0` → reject (not longer than wide)
- `size < 10 points` → reject (insufficient data)

# 5. Classification

state = "OPEN" if angle >= 80.0 else "CLOSED"

Angle is computed between the principal axis and the Y-axis `[0, 1, 0]`, clamped to `[0°, 90°]`.

Configuration Parameters

All parameters are hardcoded with sensible defaults. Modify them directly in the source files:

| Parameter | File | Default | Description |
|-----------|------|---------|-------------|
| `eps` | `clustering.py` | 0.16 m | DBSCAN neighbourhood radius |
| `min_samples` | `clustering.py` | 20 | DBSCAN minimum core points |
| `voxel_size` | `preprocess.py` | 0.05 m | Downsampling leaf size |
| `x_min/max` | `preprocess.py` | 0.0 / 10.0 | ROI X bounds (forward) |
| `y_min/max` | `preprocess.py` | -5.0 / 5.0 | ROI Y bounds (lateral) |
| `z_min/max` | `preprocess.py` | -1.0 / 5.0 | ROI Z bounds (height) |
| `angle_threshold` | `algorithm_node.py` | 80° | OPEN/CLOSED threshold |
| `timer_period` | `algorithm_node.py` | 100 ms | Processing rate (10 Hz) |
============================================================

### 6. KEY IMPLEMENTATION NOTES ###
============================================================

## Important Notes
1. Bag file path: Update the hardcoded path in `algorithm_node.py` before running.

2. Frame rate: The node publishes at 10 Hz (100 ms timer).

3. Coordinate system: Assumes LiDAR is forward-facing (X forward, Y left, Z up). The reference axis for angle computation is `[0, 1, 0]` (Y-axis).

4. Noise handling: Points labelled as noise (DBSCAN label = -1) are discarded.

5. Empty clusters: If no suitable cluster is found, the node publishes an empty MarkerArray and state = `"NO_BARRIER_ARM"`.

6. Docker environment: If running in Docker, you may need to run `xhost +` before launching RViz2 for display access.

============================================================

## License

Apache 2.0
