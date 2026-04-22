#include "barrier_detection/reader.hpp"

#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/make_shared.hpp>

#include <stdexcept>

std::vector<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>>
read_points(const std::string & bag_path, const std::string & topic)
{
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri        = bag_path;
    storage_options.storage_id = "sqlite3";

    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format  = "cdr";
    converter_options.output_serialization_format = "cdr";

    rosbag2_cpp::Reader reader;
    reader.open(storage_options, converter_options);

    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;
    std::vector<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>> frames;

    while (reader.has_next()) {
        auto bag_msg = reader.read_next();
        if (bag_msg->topic_name != topic) {
            continue;
        }

        // Deserialise the CDR blob → ROS message
        rclcpp::SerializedMessage serialized_msg(*bag_msg->serialized_data);
        sensor_msgs::msg::PointCloud2 cloud_msg;
        serializer.deserialize_message(&serialized_msg, &cloud_msg);

        // pcl::fromROSMsg expects a boost::shared_ptr cloud on PCL 1.10
        auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        pcl::fromROSMsg(cloud_msg, *cloud);
        frames.push_back(cloud);
    }

    return frames;
}
