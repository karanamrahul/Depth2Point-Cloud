# VICI-Challenge

# Depth Image to Point Cloud Conversion

This ROS2 package converts depth frames from a ROSbag file into point cloud frames and transmits them as ROS messages. The converted point cloud frames can be visualized using ROS2 Rviz.

#### Tasks



- [x] Implement the ROS2 node in C++.
- [x] Create a ROSBag of the generated point cloud messages.
- [x] Record a short video demonstrating the point cloud visualization in Rviz.
- [x] Optimize the conversion algorithm for a pipeline operating at 50 frames per second.
- [ ] Update the README with detailed instructions on setting up and using the package.
- [ ] Perform code profiling using valgrind and analyze the performance of the pipeline.

#### Data

- Dataset can be found [here](https://drive.google.com/drive/folders/1iIU8UvTj-psPHrh_VH43As0T54sNcyjG?usp=sharing)
- ROS Bag of depth frames taken from [Luxonics OAK-D S2](https://docs.luxonis.com/projects/hardware/en/latest/pages/DM9098s2.html)


# Pipeline: Depth Frame to Point Cloud Conversion and Publishing
## Overview
The pipeline described here outlines the process of converting depth frames obtained from the **"/stereo/depth"** ROS topic into a point cloud representation using camera intrinsics. The resulting point cloud is then published using the **"/point_cloud"** topic and can be saved as a ROS bag file.

## Packages Used
The pipeline relies on the following ROS packages and libraries:

- **rclcpp**: ROS 2 C++ library for creating ROS nodes and communication.
- **sensor_msgs**: ROS message package for sensor data, including the Image message.
- **cv_bridge**: ROS package for bridging between ROS messages and OpenCV images.
- **OpenCV**: Open-source computer vision library.
- **pcl**: Point Cloud Library for handling point cloud data.
- **pcl_conversions**: ROS package for converting between PCL and ROS data types.
- **pcl_io**: PCL package for reading/writing point cldepthout.mp4oud data from/to PCD files.

## Pipeline Steps
* Initialize the ROS 2 communication.
* Create a **DepthSubscriberNode**, which is a ROS node that subscribes to the **"/stereo/depth"** topic and publishes point cloud messages.
* In the DepthSubscriberNode constructor:
  - Create a subscription to the **"/stereo/depth"** topic with a callback function.
  - Create a publisher for the **"/point_cloud"** topic.
* When a depth image is received in the callback function:

  - Convert the ROS Image message to an OpenCV image using the **cv_bridge package**.
  - Convert the **Uint8 array** to **16-bit** depth values to ensure accurate representation.
  - Create a new point cloud object using the PCL library.
* Iterate over each pixel in the depth image:
  - Retrieve the depth value.
  - Convert the depth value to 3D coordinates using **camera intrinsics**.
  - Create a **pcl::PointXYZ** object and store the 3D coordinates.
  - Add the point to the point cloud.
  - Convert the **pcl::PointCloud** to a **sensor_msgs::PointCloud2** message using pcl_conversions.
- Set the frame ID from the depth image message.
- Publish the point cloud message on the **"/point_cloud"** topic.
- Save the point cloud message to a PCD file using **pcl_io**.
- Spin the **DepthSubscriberNode**, which handles callbacks and executes the main event loop.
- Shutdown the ROS 2 communication.

## Result

### Visualization:

For visualizing the generated point cloud, various tools can be used, such as:

- **Rviz**: Rviz is a powerful ROS visualization tool that allows you to visualize the point cloud data in a 3D environment. It provides features like interactive manipulation, changing viewpoints, and overlaying additional information.

- **PCL Visualizer**: The Point Cloud Library (PCL) provides a built-in visualization tool called PCL Visualizer. It offers a user-friendly interface to visualize and interact with point clouds, allowing you to explore the data from different angles and apply various visual effects.

- **Foxglove Studio**: Foxglove Studio is a comprehensive ROS visualization tool with additional analysis capabilities. It offers features like FPS monitoring, point cloud analysis tools, and other advanced functionalities that can be helpful for in-depth analysis and debugging.

Choose the visualization tool that best suits your needs and preferences. These tools enable you to explore and analyze the generated point cloud, providing valuable insights for further processing and understanding of the data.

For this demo I have used Foxglove Studio to demonstrate the performance.

### [Input Dataset - Depth Map Video](https://drive.google.com/file/d/1N0egiUlc-nIq8vDBIbeSdbcGghg6Pk-x/view?usp=drive_link)

[![Video](https://github.com/karanamrahul/VICI-Robotics-Challenge/blob/main/demo/depth_frame_4.png)](demo/depthout.mp4)
### [Output Point Cloud data Video](https://drive.google.com/file/d/1WZNBw3JCMyLUrRG0MFBoD1zdIdmyoaVD/view?usp=sharing)
[![Video](https://github.com/karanamrahul/VICI-Robotics-Challenge/blob/main/demo/pointcloud_frame1.png)](demo/pcdout.mp4)

### [Output Point Cloud data analysis Video](https://drive.google.com/file/d/1tG1eK4wGA_bhyj55AngmyTGKypO_uTwH/view?usp=drive_link)


## Optimization using OpenMP 

This document presents an analysis of the optimization techniques applied to a ROS 2 node that performs depth frame to point cloud conversion. The goal is to improve the frame rate (FPS) for the point cloud messages. The optimization technique used is OpenMP parallelization.

[Document Link](https://docs.google.com/document/d/1PMuWl9hafJ8_Ibv0K02T7-39jAyNamREKJ0xnSRZ3jQ/edit?usp=drive_link)

### [Rviz Output Data Video](https://drive.google.com/file/d/1GWhFhrDLZqJlGscDWFauQlitZwXaPMnq/view?usp=drive_link)





