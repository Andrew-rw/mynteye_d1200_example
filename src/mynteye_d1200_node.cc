#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "mynteyed/camera.h"
#include "mynteyed/utils.h"
#include "mynteyed/util/rate.h"
#include "pc_utils.h"

MYNTEYE_USE_NAMESPACE

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "mynteye_d1200_node");
  ros::NodeHandle nh;
  // Create a ROS publisher for the output point cloud
  ros::Publisher pointCloudPub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Initialize MYNTEYE camera
  Camera cam;
  DeviceInfo devInfo;
  if (!util::select(cam, &devInfo)) {
    return 1;
  }

  // Set parameters for D1200 camera
  OpenParams params(devInfo.index);
  params.color_stream_format = StreamFormat::STREAM_MJPG;
  params.depth_stream_format = StreamFormat::STREAM_YUYV;  
  params.color_mode = ColorMode::COLOR_RECTIFIED;
  params.stream_mode = StreamMode::STREAM_1280x720;
  params.ir_intensity = 10;

  cam.Open(params);

  std::cout << std::endl;
  if (!cam.IsOpened()) {
    std::cerr << "Error: Open camera failed" << std::endl;
    return 1;
  }
  std::cout << "Opened " << devInfo.name << " device." << std::endl;

  Rate rate(params.framerate);
  
  // loop
  ros::Rate loop_rate(params.framerate);
  while (nh.ok()) {
    auto cloud = pcutil::get_point_cloud(&cam, 500.0);
    if (cloud) {
      sensor_msgs::PointCloud2 msg;
      pcl::toROSMsg(*cloud, msg);
      msg.header.stamp = ros::Time().now();
      msg.header.frame_id = "depth_frame";

      pointCloudPub.publish (msg);
    }
    loop_rate.sleep();
  }

  cam.Close();
  return 0;
}