#include "barrier_detection/reader.hpp"
#include "barrier_detection/preprocess.hpp"
#include "barrier_detection/clustering.hpp"
#include "barrier_detection/geometry.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/shared_ptr.hpp>

#include <Eigen/Dense>
#include <cmath>
#include <string>
#include <vector>
#include <algorithm>
#include <iostream>
#include <iomanip>

// ============================================================================
// Data types
// ============================================================================

struct ClusterFeatures
{
    float           length;
    float           width;
    float           height;
    float           lw_ratio;
    float           linearity;
    Eigen::Vector3f direction;  ///< principal PCA axis (unit vector)
};

struct BestClusterResult
{
    // PCL 1.10 uses boost::shared_ptr — must match the type returned by
    // cluster_dbscan() and crop_roi() or you get an implicit-conversion error.
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud;  // nullptr if none
    float           distance;
    Eigen::Vector3f center;
    ClusterFeatures features;
};

// ============================================================================
// LidarProcessingNode
// ============================================================================

class LidarProcessingNode : public rclcpp::Node
{
public:
    explicit LidarProcessingNode(const std::string & bag_path)
    : Node("lidar_processing_node"), frame_idx_(0)
    {
        // ── Publishers ────────────────────────────────────────────────────────
        raw_pub_       = create_publisher<sensor_msgs::msg::PointCloud2>("/lidar_raw",        10);
        filtered_pub_  = create_publisher<sensor_msgs::msg::PointCloud2>("/lidar_filtered",   10);
        clusters_pub_  = create_publisher<visualization_msgs::msg::MarkerArray>("/lidar_clusters",   10);
        direction_pub_ = create_publisher<visualization_msgs::msg::Marker>("/barrier_direction", 10);
        state_pub_     = create_publisher<std_msgs::msg::String>("/barrier_state",  10);
        angle_pub_     = create_publisher<std_msgs::msg::Float32>("/barrier_angle", 10);

        RCLCPP_INFO(get_logger(), "Loading data from: %s", bag_path.c_str());

        try {
            frames_ = read_points(bag_path);
            RCLCPP_INFO(get_logger(), "Successfully loaded %zu frames", frames_.size());

            // 10 Hz timer — same as Python
            timer_ = create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&LidarProcessingNode::process_and_publish, this));
        } catch (const std::exception & e) {
            RCLCPP_ERROR(get_logger(), "Error loading data: %s", e.what());
        }
    }

private:
    // ROS publishers — these remain std::shared_ptr (rclcpp's own type)
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr       raw_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr       filtered_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr clusters_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr     direction_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr               state_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr              angle_pub_;
    rclcpp::TimerBase::SharedPtr                                      timer_;

    // PCL clouds — must be boost::shared_ptr to match PCL 1.10 internals
    std::vector<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>> frames_;
    std::size_t frame_idx_;

    static constexpr const char * FRAME_ID = "lidar_frame3";

    // ── to_ros_cloud ──────────────────────────────────────────────────────────
    /**
     * Convert a PCL cloud to a ROS PointCloud2 message.
     * Accepts boost::shared_ptr (PCL 1.10 native pointer type).
     */
    template<typename PointT>
    sensor_msgs::msg::PointCloud2
    to_ros_cloud(const boost::shared_ptr<pcl::PointCloud<PointT>> & cloud,
                 const std::string & frame_id = FRAME_ID)
    {
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*cloud, msg);
        msg.header.stamp    = now();
        msg.header.frame_id = frame_id;
        return msg;
    }

    // ── get_cluster_features ─────────────────────────────────────────────────
    /**
     * Compute PCA-based features for a single cluster.
     * Mirrors Python: LidarProcessingNode.get_cluster_features()
     *
     * Uses fit_line_pca() which already handles:
     *   - covariance C = XᵀX/(n-1)
     *   - descending eigenvalue sort
     *   - λ₁ stabilisation ≥ 1e-3
     */
    ClusterFeatures get_cluster_features(const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> & c)
    {
        ClusterFeatures f{};
        if (!c || static_cast<int>(c->size()) < 5) { return f; }

        PCAResult pca = fit_line_pca(c);

        // ── Length — projection range on principal axis ──────────────────────
        float proj_min =  1e9f, proj_max = -1e9f;
        float w_min    =  1e9f, w_max   = -1e9f;
        float z_min    =  1e9f, z_max   = -1e9f;

        for (const auto & pt : c->points) {
            const Eigen::Vector3f p(pt.x - pca.centroid.x(),
                                    pt.y - pca.centroid.y(),
                                    pt.z - pca.centroid.z());
            const float p0 = p.dot(pca.direction);
            const float p1 = p.dot(pca.direction2);

            proj_min = std::min(proj_min, p0);
            proj_max = std::max(proj_max, p0);
            w_min    = std::min(w_min,    p1);
            w_max    = std::max(w_max,    p1);
            z_min    = std::min(z_min,    pt.z);
            z_max    = std::max(z_max,    pt.z);
        }

        f.length    = proj_max - proj_min;
        f.width     = w_max - w_min;
        f.height    = z_max - z_min;
        f.linearity = pca.linearity;
        f.direction = pca.direction;

        // Stabilise width (matches Python: max(width, 1e-3))
        f.width    = std::max(f.width, 1e-3f);
        f.lw_ratio = f.length / (f.width + 1e-6f);

        return f;
    }

    // ── get_cluster_distance ─────────────────────────────────────────────────
    /** Euclidean distance from sensor origin to cluster centroid. */
    std::pair<float, Eigen::Vector3f>
    get_cluster_distance(const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> & c)
    {
        Eigen::Vector3f center = Eigen::Vector3f::Zero();
        for (const auto & pt : c->points) {
            center += Eigen::Vector3f(pt.x, pt.y, pt.z);
        }
        center /= static_cast<float>(c->size());
        return {center.norm(), center};
    }

    // ── select_best_cluster ──────────────────────────────────────────────────
    /**
     * Find the thinnest AND longest cluster.
     * Score = length / (width + ε) + 0.5 * length
     *
     * Hard constraints (mirrors Python):
     *   width  > 0.5 m  → skip
     *   lw_ratio < 1.0  → skip
     *   size   < 10 pts → skip
     */
    BestClusterResult select_best_cluster(
        const std::vector<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> & clusters)
    {
        BestClusterResult best;
        best.cloud    = nullptr;
        best.distance = 0.0f;
        best.center   = Eigen::Vector3f::Zero();

        float best_score = -1.0f;

        for (const auto & c : clusters) {
            if (!c || static_cast<int>(c->size()) < 10) { continue; }

            ClusterFeatures f = get_cluster_features(c);

            // Hard constraints for barrier arm
            if (f.width > 0.5f)   { continue; }
            if (f.lw_ratio < 1.0f){ continue; }

            const float score = f.length / (f.width + 1e-6f) + 0.5f * f.length;

            if (score > best_score) {
                best_score     = score;
                best.cloud     = c;
                best.features  = f;
                auto [dist, ct] = get_cluster_distance(c);
                best.distance  = dist;
                best.center    = ct;
            }
        }

        return best;
    }

    // ── cluster_to_markers ───────────────────────────────────────────────────
    /**
     * Build a MarkerArray for the selected cluster (bounding box + text label).
     * Mirrors Python: LidarProcessingNode.cluster_to_markers()
     */
    visualization_msgs::msg::MarkerArray
    cluster_to_markers(const BestClusterResult & best)
    {
        visualization_msgs::msg::MarkerArray ma;
        if (!best.cloud) { return ma; }

        const auto & c  = best.cloud;
        const auto & f  = best.features;
        const auto & ct = best.center;

        // ── Bounding-box cube ─────────────────────────────────────────────────
        float x_min =  1e9f, x_max = -1e9f;
        float y_min =  1e9f, y_max = -1e9f;
        float z_min =  1e9f, z_max = -1e9f;

        for (const auto & pt : c->points) {
            x_min = std::min(x_min, pt.x); x_max = std::max(x_max, pt.x);
            y_min = std::min(y_min, pt.y); y_max = std::max(y_max, pt.y);
            z_min = std::min(z_min, pt.z); z_max = std::max(z_max, pt.z);
        }

        visualization_msgs::msg::Marker box;
        box.header.frame_id = FRAME_ID;
        box.header.stamp    = now();
        box.ns              = "best_cluster";
        box.id              = 0;
        box.type            = visualization_msgs::msg::Marker::CUBE;
        box.action          = visualization_msgs::msg::Marker::ADD;

        box.pose.position.x = static_cast<double>(ct.x());
        box.pose.position.y = static_cast<double>(ct.y());
        box.pose.position.z = static_cast<double>(ct.z());
        box.pose.orientation.w = 1.0;

        box.scale.x = static_cast<double>(std::max(x_max - x_min, 0.3f));
        box.scale.y = static_cast<double>(std::max(y_max - y_min, 0.3f));
        box.scale.z = static_cast<double>(std::max(z_max - z_min, 0.2f));

        box.color.r = 0.0f; box.color.g = 1.0f;
        box.color.b = 0.0f; box.color.a = 0.7f;

        box.lifetime = rclcpp::Duration::from_seconds(0.1);
        ma.markers.push_back(box);

        // ── Text label ────────────────────────────────────────────────────────
        visualization_msgs::msg::Marker txt;
        txt.header.frame_id = FRAME_ID;
        txt.header.stamp    = now();
        txt.ns              = "cluster_labels";
        txt.id              = 1000;
        txt.type            = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        txt.action          = visualization_msgs::msg::Marker::ADD;

        txt.pose.position.x = static_cast<double>(ct.x());
        txt.pose.position.y = static_cast<double>(ct.y());
        txt.pose.position.z = static_cast<double>(ct.z()) + 0.4;
        txt.pose.orientation.w = 1.0;
        txt.scale.z         = 0.35;

        txt.color.r = 1.0f; txt.color.g = 1.0f;
        txt.color.b = 1.0f; txt.color.a = 1.0f;

        // Build label string
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(2)
           << "ARM (" << c->size() << " pts"
           << ", L=" << f.length << "m"
           << ", W=" << f.width  << "m"
           << ", L/W=" << std::setprecision(1) << f.lw_ratio
           << ", D="   << best.distance << "m)";
        txt.text     = ss.str();
        txt.lifetime = rclcpp::Duration::from_seconds(0.1);
        ma.markers.push_back(txt);

        return ma;
    }

    // ── create_direction_marker ───────────────────────────────────────────────
    /**
     * Arrow marker showing the principal axis of the selected cluster.
     * Mirrors Python: LidarProcessingNode.create_direction_marker()
     */
    visualization_msgs::msg::Marker
    create_direction_marker(const BestClusterResult & best)
    {
        visualization_msgs::msg::Marker marker;
        if (!best.cloud) { return marker; }

        const Eigen::Vector3f & dir = best.features.direction;
        const Eigen::Vector3f & ct  = best.center;

        const float arrow_len = std::clamp(best.features.length, 0.5f, 2.0f);

        marker.header.frame_id = FRAME_ID;
        marker.header.stamp    = now();
        marker.ns              = "direction";
        marker.id              = 0;
        marker.type            = visualization_msgs::msg::Marker::ARROW;
        marker.action          = visualization_msgs::msg::Marker::ADD;

        geometry_msgs::msg::Point start, end;
        start.x = static_cast<double>(ct.x());
        start.y = static_cast<double>(ct.y());
        start.z = static_cast<double>(ct.z());

        end.x = static_cast<double>(ct.x() + dir.x() * arrow_len);
        end.y = static_cast<double>(ct.y() + dir.y() * arrow_len);
        end.z = static_cast<double>(ct.z());       // keep arrow in XY plane

        marker.points = {start, end};
        marker.scale.x = 0.08;
        marker.scale.y = 0.15;
        marker.scale.z = 0.15;

        marker.color.r = 1.0f; marker.color.g = 0.0f;
        marker.color.b = 0.0f; marker.color.a = 1.0f;

        marker.lifetime = rclcpp::Duration::from_seconds(0.1);
        return marker;
    }

    // ── process_and_publish (timer callback) ─────────────────────────────────
    /**
     * Main processing loop, called at 10 Hz.
     * Mirrors Python: LidarProcessingNode.process_and_publish()
     */
    void process_and_publish()
    {
        if (frames_.empty()) { return; }

        if (frame_idx_ >= frames_.size()) {
            frame_idx_ = 0;
            RCLCPP_INFO(get_logger(), "Loop completed, starting over");
        }

        const auto & raw_cloud = frames_[frame_idx_];

        // ── Publish raw points ─────────────────────────────────────────────
        raw_pub_->publish(to_ros_cloud<pcl::PointXYZI>(raw_cloud));

        try {
            // ── Preprocess ─────────────────────────────────────────────────
            auto filtered = crop_roi(raw_cloud);
            filtered_pub_->publish(to_ros_cloud<pcl::PointXYZ>(filtered));

            // ── Cluster ────────────────────────────────────────────────────
            auto clusters = cluster_dbscan(filtered);   // eps=0.16, min_samples=20

            std::cout << "\n" << std::string(60, '=') << "\n"
                      << "FRAME " << frame_idx_ << ":\n"
                      << "  Total clusters: " << clusters.size() << "\n";

            // ── Select best ────────────────────────────────────────────────
            BestClusterResult best = select_best_cluster(clusters);

            if (best.cloud) {
                const ClusterFeatures & f = best.features;

                std::cout << std::fixed << std::setprecision(2)
                          << "  BEST CLUSTER: " << best.cloud->size() << " pts\n"
                          << "    Length:    " << f.length    << " m\n"
                          << "    Width:     " << f.width     << " m\n"
                          << "    L/W ratio: " << f.lw_ratio  << "\n"
                          << "    Linearity: " << f.linearity << "\n"
                          << "    Distance:  " << best.distance << " m\n";

                // Publish cluster bounding box + label
                clusters_pub_->publish(cluster_to_markers(best));

                // Compute and publish angle
                const float angle = compute_angle(f.direction);

                std_msgs::msg::Float32 angle_msg;
                angle_msg.data = angle;
                angle_pub_->publish(angle_msg);

                // Publish direction arrow
                direction_pub_->publish(create_direction_marker(best));

                // Classify — mirrors Python: "OPEN" if angle >= 80°
                const std::string state_str = (angle >= 80.0f) ? "OPEN" : "CLOSED";

                std_msgs::msg::String state_msg;
                state_msg.data = state_str;
                state_pub_->publish(state_msg);

                std::cout << std::setprecision(1)
                          << "  RESULT: Angle=" << angle << "°, State=" << state_str << "\n";

                RCLCPP_INFO(get_logger(),
                    "Frame %zu: %zu pts, L=%.2fm, W=%.2fm, Angle=%.1f°",
                    frame_idx_, best.cloud->size(), f.length, f.width, angle);

            } else {
                // No valid cluster
                clusters_pub_->publish(visualization_msgs::msg::MarkerArray{});

                std_msgs::msg::String state_msg;
                state_msg.data = "NO_BARRIER_ARM";
                state_pub_->publish(state_msg);

                std::cout << "  RESULT: No suitable cluster found\n";
            }

            std::cout << std::string(60, '=') << "\n";

        } catch (const std::exception & e) {
            RCLCPP_WARN(get_logger(), "Error: %s", e.what());
        }

        ++frame_idx_;
    }
};

// ============================================================================
// main
// ============================================================================

int main(int argc, char * argv[])
{
    const std::string bag_path = "/root/Desktop/rosbag2_lidar_data";

    rclcpp::init(argc, argv);

    std::cout << std::string(60, '=') << "\n"
              << "BARRIER ARM DETECTION — C++ PORT\n"
              << std::string(60, '=') << "\n"
              << "Strategy : publish the thinnest AND longest cluster\n"
              << "Score    : length/width + 0.5*length\n"
              << "Rewards  : thin (small width) + long (large length)\n"
              << std::string(60, '=') << "\n"
              << "Press Ctrl+C to stop\n\n";

    auto node = std::make_shared<LidarProcessingNode>(bag_path);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
