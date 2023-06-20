/************╔══════════════════════════════╗************
  *              *    🌟   PIPELINE    🌟       *              
  *              *   Depth Frame to Point Cloud   *            
  *              *       Conversion       *              
  ************╚══════════════════════════════╝************/


/* 
@author Rahul Karanam 
@date June 18, 2023
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
#include <omp.h>                                 // OpenMP header

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

    // Initialize the start time for FPS calculation
    start_time_ = std::chrono::steady_clock::now();
  }

private:
  /**
   * @brief Callback function for processing the depth image.
   * @param msg The received depth image message.
   */
  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    // Increment the frame count
    ++frame_count_;

    // Check if it's time to publish the point cloud
    if (frame_count_ % POINT_CLOUD_PUBLISH_INTERVAL != 0) {
      return;  // Skip this frame
    }

    // Convert the ROS Image message to an OpenCV image
    cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);

    // Convert the Uint8 array to 16-bit depth values
    cv::Mat depth_image = cv_image->image;
    depth_image.convertTo(depth_image, CV_16UC1);

    // Store depth values as a point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    point_cloud->width = depth_image.cols;
    point_cloud->height = depth_image.rows;

    // Set the number of threads to use
    int num_threads = omp_get_max_threads();

    // Calculate the number of rows per thread
    int rows_per_thread = depth_image.rows / num_threads;

    // Calculate the total number of points in the point cloud
    int total_points = depth_image.rows * depth_image.cols;

    // Preallocate memory for the point cloud
    point_cloud->reserve(total_points);

    // Iterate over each thread
#pragma omp parallel num_threads(num_threads)
    {
      // Calculate the start and end row indices for this thread
      int thread_id = omp_get_thread_num();
      int start_row = thread_id * rows_per_thread;
      int end_row = (thread_id == num_threads - 1) ? depth_image.rows : (thread_id + 1) * rows_per_thread;

      // Iterate over the assigned range of rows
      for (int row = start_row; row < end_row; ++row) {
        // Iterate over each column in the row
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
    }

    // Convert the pcl::PointCloud to a sensor_msgs::PointCloud2 message
    sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg(new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(*point_cloud, *cloud_msg);

    // Set the frame ID from the depth image message
    cloud_msg->header.frame_id = msg->header.frame_id;

    // Publish the point cloud message
    point_cloud_pub_->publish(*cloud_msg);

    // Calculate and print the FPS every second
    ++fps_count_;
    if (std::chrono::steady_clock::now() - start_time_ >= std::chrono::seconds(1)) {
      double elapsed_seconds = std::chrono::duration_cast<std::chrono::duration<double>>(
        std::chrono::steady_clock::now() - start_time_).count();
      double fps = static_cast<double>(fps_count_) / elapsed_seconds;

      RCLCPP_INFO(get_logger(), "Current FPS: %.2f", fps);

      // Reset the counters for the next measurement
      start_time_ = std::chrono::steady_clock::now();
      fps_count_ = 0;
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;

  // Camera intrinsic matrix (resized to camera resolution)
  cv::Mat intrinsic_matrix_ = (cv::Mat_<float>(3, 3) << 400.00485229, 0, 323.98687744,
                               0, 400.00485229, 200.87487793,
                               0, 0, 1);

  int frame_count_ = 0;  // Frame count variable

  // Define the desired frame rate for point cloud publication
  const int POINT_CLOUD_PUBLISH_INTERVAL = 60 / 50;  // Publish every 60/50 frames

  // Variables for FPS calculation
  std::chrono::time_point<std::chrono::steady_clock> start_time_;
  int fps_count_ = 0;
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
