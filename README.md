# LiDAR_barrier_arm
A ROS2 pipeline for real-time LiDAR-based barrier arm detection, implemented in both Python and C++. Processes 3D point cloud data from a RosBag using ROI cropping, statistical outlier removal, DBSCAN clustering, and PCA-based geometry to classify barrier arm state (open/closed) at 10 Hz.
