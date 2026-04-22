#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/shared_ptr.hpp>

/**
 * Full preprocessing pipeline (mirrors Python preprocess.crop_roi):
 *   1) ROI box crop + ground removal via z_min
 *   2) Statistical outlier removal (k=8, z_thresh=2.5)  [optional]
 *   3) Voxel downsampling (voxel_size=0.05 m)
 *
 * Returns an Nx3 float cloud (PointXYZ — intensity not needed downstream).
 *
 * NOTE: PCL 1.10 on Ubuntu 20.04 uses boost::shared_ptr internally.
 *       All PCL cloud pointers must be boost::shared_ptr to avoid
 *       implicit-conversion compile errors.
 */
boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>
crop_roi(const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> & input,
         float x_min = 0.0f,  float x_max = 10.0f,
         float y_min = -5.0f, float y_max =  5.0f,
         float z_min = -1.0f, float z_max =  5.0f,
         float voxel_size = 0.05f,
         bool  do_outlier_removal = true);
