/************╔══════════════════════════════╗************
  *              *    🌟   PIPELINE    🌟       *              
  *              *   Depth Frame to Point Cloud   *            
  *              *       Conversion       *              
  ************╚══════════════════════════════╝************/


/* 
@author Rahul Karanam 
@date June 18 2023
@brief Depth Frame to Point Cloud Conversion
@description This node subscribes to the "/stereo/depth" topic and converts the depth image to a point cloud.
@org Vici Robotics
*/


// ROS 2 libraries
#include <rclcpp/rclcpp.hpp>                     // ROS 2 C++ library
#include <sensor_msgs/msg/image.hpp>             // ROS 2 Image message
#include <sensor_msgs/msg/point_cloud2.hpp>      // ROS 2 PointCloud2 message
#include <cv_bridge/cv_bridge.h>                 // OpenCV ROS bridge
#include <opencv2/opencv.hpp>                    // OpenCV library
#include <pcl/point_types.h>                     // PCL point types
#include <pcl/point_cloud.h>                     // PCL point cloud
#include <pcl_conversions/pcl_conversions.h>     // PCL ROS conversions
#include <pcl/io/pcd_io.h>                       // PCL PCD file IO

class DepthSubscriberNode : public rclcpp::Node {
public:
  /**
   * @brief Constructs a DepthSubscriberNode object.
   */
  DepthSubscriberNode() : Node("depth_visualizer_node") {
    // Create a subscription to the "/stereo/depth" topic with a callback function
    subscription_ = create_subscription<sensor_msgs::msg::Image>(
      "/stereo/depth", 10, std::bind(&DepthSubscriberNode::depthCallback, this, std::placeholders::_1));

    // Create a publisher for the "/point_cloud" topic
    point_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/point_cloud", 10);
  }

private:
  /**
   * @brief Callback function for processing the depth image.
   * @param msg The received depth image message.
   */
  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    // Convert the ROS Image message to an OpenCV image
    cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);

    // Convert the Uint8 array to 16-bit depth values
    cv::Mat depth_image = cv_image->image;
    depth_image.convertTo(depth_image, CV_16UC1);

    // Store depth values as a point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Iterate over each pixel in the depth image
    for (int row = 0; row < depth_image.rows; ++row) {
      for (int col = 0; col < depth_image.cols; ++col) {
        uint16_t depth_value = depth_image.at<uint16_t>(row, col);

        // Convert depth value to 3D coordinates using camera intrinsics
        float x = (col - intrinsic_matrix_.at<float>(0, 2)) * depth_value / intrinsic_matrix_.at<float>(0, 0);
        float y = (row - intrinsic_matrix_.at<float>(1, 2)) * depth_value / intrinsic_matrix_.at<float>(1, 1);
        float z = depth_value;

        // Create a pcl::PointXYZ object and store the coordinates
        pcl::PointXYZ point;
        point.x = x;
        point.y = y;
        point.z = z;

        // Add the point to the point cloud
        point_cloud->push_back(point);
      }
    }

    // Convert the pcl::PointCloud to a sensor_msgs::PointCloud2 message
    sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg(new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(*point_cloud, *cloud_msg);

    // Set the frame ID from the depth image message
    cloud_msg->header.frame_id = msg->header.frame_id;

    // Publish the point cloud message
    point_cloud_pub_->publish(*cloud_msg);

    // Save the point cloud message to a PCD file
    pcl::io::savePCDFileBinary("point_cloud.pcd", *point_cloud);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;

  // Camera intrinsic matrix (resized to camera resolution)
  cv::Mat intrinsic_matrix_ = (cv::Mat_<float>(3, 3) << 400.00485229, 0, 323.98687744,
                               0, 400.00485229, 200.87487793,
                               0, 0, 1);
};

int main(int argc, char** argv) {
  // Initialize the ROS 2 communication
  rclcpp::init(argc, argv);

  // Create a shared pointer to the DepthSubscriberNode object
  auto node = std::make_shared<DepthSubscriberNode>();

  // Spin the node, handling callbacks and executing the main event loop
  rclcpp::spin(node);

  // Shutdown the ROS 2 communication
  rclcpp::shutdown();

  return 0;
}
