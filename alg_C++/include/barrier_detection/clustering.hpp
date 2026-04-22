#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/shared_ptr.hpp>
#include <vector>

/**
 * DBSCAN clustering on a PointXYZ cloud.
 *
 * Mirrors Python: clustering.cluster(points, eps=0.16, min_samples=20)
 *
 * Uses a KD-tree for radius search — O(n log n) neighbour queries.
 * Noise points (label == -1) are discarded; only valid clusters returned.
 *
 * NOTE: PCL 1.10 on Ubuntu 20.04 uses boost::shared_ptr internally.
 *
 * @param eps         Neighbourhood radius in metres (default 0.16)
 * @param min_samples Minimum points to form a core point (default 20)
 * @return            Vector of point clouds, one per cluster
 */
std::vector<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>>
cluster_dbscan(const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> & cloud,
               float eps         = 0.16f,
               int   min_samples = 20);
