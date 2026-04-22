#pragma once

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/shared_ptr.hpp>

/**
 * Result of a full 3-axis PCA decomposition.
 * All three eigenvectors / eigenvalues are stored so callers can compute
 * both length (axis 0) and width (axis 1) projections without re-running PCA.
 */
struct PCAResult
{
    Eigen::Vector3f direction;   ///< Principal axis (largest eigenvalue), unit vec
    Eigen::Vector3f direction2;  ///< Second axis (second-largest eigenvalue)
    Eigen::Vector3f centroid;    ///< Point-cloud centroid
    Eigen::Vector3f eigenvalues; ///< Sorted descending: [λ0, λ1, λ2]
    float           linearity;   ///< λ0 / (λ1 + ε)
};

/**
 * Robust PCA line fitting over a PointXYZ cloud.
 * Mirrors Python: geometry.fit_line_pca(points)
 *
 * Extras vs the Python version:
 *   - Returns direction2 (second principal axis, needed for width computation)
 *   - Enforces consistent direction sign (direction.x() >= 0)
 */
PCAResult fit_line_pca(const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> & cloud);

/**
 * Angle (degrees, in [0, 90]) between `direction` and `reference`.
 * Mirrors Python: geometry.compute_angle(direction, reference_axis=[0,1,0])
 */
float compute_angle(const Eigen::Vector3f & direction,
                    const Eigen::Vector3f & reference = Eigen::Vector3f(0.0f, 1.0f, 0.0f));
