#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>

class PointCloudSplitterPCL : public rclcpp::Node {
public:
    PointCloudSplitterPCL() : Node("point_cloud_splitter_pcl") {
        // Subscribe to the PointCloud2 topic
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "input_pointcloud", 10, std::bind(&PointCloudSplitterPCL::pointCloudCallback, this, std::placeholders::_1));

        // Create publishers for split point clouds
        pub_lower_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("lower_pointcloud", 10);
        pub_higher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("higher_pointcloud", 10);
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // Convert ROS PointCloud2 message to PCL point cloud
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*msg, pcl_pc2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

        // Split the point cloud based on height value (z-coordinate)
        pcl::PointCloud<pcl::PointXYZ>::Ptr lower_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr higher_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        float split_height = 0.5; // Set the split height value here

        for (const auto& point : cloud->points) {
            if (point.z < split_height) {
                lower_cloud->points.push_back(point);
            } else {
                higher_cloud->points.push_back(point);
            }
        }

        // Convert PCL point clouds back to ROS PointCloud2 messages
        sensor_msgs::msg::PointCloud2 lower_msg, higher_msg;
        pcl::toROSMsg(*lower_cloud, lower_msg);
        pcl::toROSMsg(*higher_cloud, higher_msg);
        lower_msg.header = msg->header;
        higher_msg.header = msg->header;

        // Publish the split point clouds on separate topics
        pub_lower_->publish(lower_msg);
        pub_higher_->publish(higher_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_lower_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_higher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudSplitterPCL>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
