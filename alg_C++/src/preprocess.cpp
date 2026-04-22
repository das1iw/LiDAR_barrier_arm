#include "barrier_detection/preprocess.hpp"

#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/io.h>
#include <boost/make_shared.hpp>

// PCL 1.10 (Ubuntu 20.04) defines PointCloud::Ptr as boost::shared_ptr.
// All setInputCloud() calls require boost::shared_ptr — std::shared_ptr
// will not implicitly convert and causes a compile error.

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

using CloudXYZ  = pcl::PointCloud<pcl::PointXYZ>;
using CloudXYZI = pcl::PointCloud<pcl::PointXYZI>;

/** ROI box filter applied in-place along one named field. */
static void passthrough(boost::shared_ptr<CloudXYZ> & cloud,
                        const std::string & field, float lo, float hi)
{
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);          // expects boost::shared_ptr
    pass.setFilterFieldName(field);
    pass.setFilterLimits(lo, hi);
    pass.filter(*cloud);
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

boost::shared_ptr<CloudXYZ>
crop_roi(const boost::shared_ptr<CloudXYZI> & input,
         float x_min, float x_max,
         float y_min, float y_max,
         float z_min, float z_max,
         float voxel_size,
         bool  do_outlier_removal)
{
    // ── 1) Convert XYZI → XYZ ───────────────────────────────────────────────
    auto cloud = boost::make_shared<CloudXYZ>();
    pcl::copyPointCloud(*input, *cloud);

    if (cloud->empty()) { return cloud; }

    // ── 2) ROI crop + ground removal ────────────────────────────────────────
    passthrough(cloud, "x", x_min, x_max);
    if (cloud->empty()) { return cloud; }
    passthrough(cloud, "y", y_min, y_max);
    if (cloud->empty()) { return cloud; }
    passthrough(cloud, "z", z_min, z_max);   // z_min=-1 removes ground
    if (cloud->empty()) { return cloud; }

    // ── 3) Statistical outlier removal (k=8, z_thresh=2.5) ─────────────────
    if (do_outlier_removal) {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud);           // expects boost::shared_ptr
        sor.setMeanK(8);
        sor.setStddevMulThresh(2.5);
        sor.filter(*cloud);
        if (cloud->empty()) { return cloud; }
    }

    // ── 4) Voxel downsampling ───────────────────────────────────────────────
    if (voxel_size > 0.0f) {
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(cloud);            // expects boost::shared_ptr
        vg.setLeafSize(voxel_size, voxel_size, voxel_size);
        vg.filter(*cloud);
    }

    return cloud;
}
