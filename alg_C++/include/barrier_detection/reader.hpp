#pragma once

#include <string>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/shared_ptr.hpp>

/**
 * Read all PointCloud2 messages from a ROS2 bag file on the given topic.
 * Returns one PCL cloud per message, preserving intensity via PointXYZI.
 *
 * NOTE: PCL 1.10 on Ubuntu 20.04 uses boost::shared_ptr internally.
 *
 * Mirrors Python: reader.read_points(bag_path, topic)
 */
std::vector<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>>
read_points(const std::string & bag_path,
            const std::string & topic = "/front/rslidar_points");
