#include "barrier_detection/geometry.hpp"

#include <Eigen/Eigenvalues>
#include <boost/shared_ptr.hpp>
#include <cmath>

// ---------------------------------------------------------------------------
// fit_line_pca
// ---------------------------------------------------------------------------
// Direct port of Python geometry.fit_line_pca().
//
// Key differences from a generic PCA:
//   • Uses the biased-corrected covariance  C = XᵀX / (n-1)  (numpy default)
//   • Clamps λ₁ ≥ 1e-3 to stabilise motion/noise
//   • Enforces direction.x() ≥ 0 for temporal consistency
// ---------------------------------------------------------------------------

PCAResult fit_line_pca(const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> & cloud)
{
    PCAResult result;
    result.direction   = Eigen::Vector3f::Zero();
    result.direction2  = Eigen::Vector3f::Zero();
    result.centroid    = Eigen::Vector3f::Zero();
    result.eigenvalues = Eigen::Vector3f::Zero();
    result.linearity   = 0.0f;

    if (!cloud || static_cast<int>(cloud->size()) < 5) {
        return result;
    }

    const int n = static_cast<int>(cloud->size());

    // ── Centroid ─────────────────────────────────────────────────────────────
    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    for (const auto & pt : cloud->points) {
        centroid += Eigen::Vector3f(pt.x, pt.y, pt.z);
    }
    centroid /= static_cast<float>(n);
    result.centroid = centroid;

    // ── Centred data matrix  [n × 3] ─────────────────────────────────────────
    Eigen::MatrixXf X(n, 3);
    for (int i = 0; i < n; ++i) {
        X(i, 0) = cloud->points[i].x - centroid.x();
        X(i, 1) = cloud->points[i].y - centroid.y();
        X(i, 2) = cloud->points[i].z - centroid.z();
    }

    // ── Covariance  C = XᵀX / (n-1) ─────────────────────────────────────────
    Eigen::Matrix3f cov = (X.transpose() * X) / static_cast<float>(n - 1);

    // ── Eigen decomposition (SelfAdjointEigenSolver returns ascending order) ─
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
    Eigen::Vector3f eigvals = solver.eigenvalues();   // ascending: λ₀ ≤ λ₁ ≤ λ₂
    Eigen::Matrix3f eigvecs = solver.eigenvectors();  // cols correspond to eigvals

    // Reverse to descending (matches np.argsort(eigvals)[::-1])
    Eigen::Vector3f eigvals_desc;
    eigvals_desc << eigvals(2), eigvals(1), eigvals(0);

    Eigen::Matrix3f eigvecs_desc;
    eigvecs_desc.col(0) = eigvecs.col(2);  // principal
    eigvecs_desc.col(1) = eigvecs.col(1);  // second
    eigvecs_desc.col(2) = eigvecs.col(0);  // third

    // ── Stabilise λ₁ ─────────────────────────────────────────────────────────
    if (eigvals_desc(1) < 1e-3f) { eigvals_desc(1) = 1e-3f; }
    result.eigenvalues = eigvals_desc;

    // ── Principal direction ───────────────────────────────────────────────────
    Eigen::Vector3f dir = eigvecs_desc.col(0);
    const float norm = dir.norm();
    if (norm < 1e-6f) { return result; }
    dir /= norm;

    // Enforce consistent sign (reduces angle flipping across frames)
    if (dir.x() < 0.0f) { dir = -dir; }
    result.direction = dir;

    // ── Second direction (for width projection in the node) ───────────────────
    Eigen::Vector3f dir2 = eigvecs_desc.col(1);
    const float norm2 = dir2.norm();
    if (norm2 > 1e-6f) { dir2 /= norm2; }
    result.direction2 = dir2;

    // ── Linearity ─────────────────────────────────────────────────────────────
    result.linearity = eigvals_desc(0) / (eigvals_desc(1) + 1e-6f);

    return result;
}

// ---------------------------------------------------------------------------
// compute_angle
// ---------------------------------------------------------------------------
// Mirrors Python geometry.compute_angle(direction, reference_axis=[0,1,0]).
// Returns angle in [0, 90] degrees (orientation-invariant via |dot|).
// ---------------------------------------------------------------------------

float compute_angle(const Eigen::Vector3f & direction,
                    const Eigen::Vector3f & reference)
{
    const float dir_norm = direction.norm();
    const float ref_norm = reference.norm();

    if (dir_norm < 1e-6f || ref_norm < 1e-6f) { return 0.0f; }

    const Eigen::Vector3f d = direction / dir_norm;
    const Eigen::Vector3f r = reference / ref_norm;

    const float cos_theta = std::clamp(std::abs(d.dot(r)), -1.0f, 1.0f);
    return std::acos(cos_theta) * 180.0f / static_cast<float>(M_PI);
}
