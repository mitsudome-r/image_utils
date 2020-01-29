#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>

#include <vector>
#include <string>


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "camera_info_publisher");


  std::vector<double> camera_matrix;
  std::vector<double> distortion_coefficients;
  std::vector<double> projection_matrix;
  int image_width;
  int image_height;
  std::string frame_id;
  std::string distortion_model;

  // receive parameters
  ros::NodeHandle pnh("~");
  if (!pnh.hasParam("camera_matrix/data"))
  {
    ROS_ERROR("missing camera_matrix!!");
    std::exit(1);
  }
  if (!pnh.hasParam("distortion_coefficients/data"))
  {
    ROS_ERROR("missing distortion_coefficients!!");
    std::exit(1);
  }
  if (!pnh.hasParam("projection_matrix/data"))
  {
    ROS_ERROR("missing projection_matrix!!");
    std::exit(1);
  }

  pnh.param<std::vector<double>>("camera_matrix/data", camera_matrix, camera_matrix);
  pnh.param<std::vector<double>>("distortion_coefficients/data", distortion_coefficients, distortion_coefficients);
  pnh.param<std::vector<double>>("projection_matrix/data", projection_matrix, projection_matrix);
  pnh.param<int>("image_height", image_height, 0);
  pnh.param<int>("image_width", image_width, 0);
  pnh.param<std::string>("frame_id", frame_id, "camera");
  pnh.param<std::string>("distortion_model", distortion_model, "plum_blob");

  // create message
  sensor_msgs::CameraInfo camera_info;
  camera_info.header.stamp = ros::Time::now();
  camera_info.header.frame_id = frame_id;
  camera_info.height = image_height;
  camera_info.width = image_width;
  camera_info.distortion_model = distortion_model;
  camera_info.D = distortion_coefficients;
  camera_info.K[0] = camera_matrix.at(0);
  camera_info.K[1] = camera_matrix.at(1);
  camera_info.K[2] = camera_matrix.at(2);
  camera_info.K[3] = camera_matrix.at(3);
  camera_info.K[4] = camera_matrix.at(4);
  camera_info.K[5] = camera_matrix.at(5);
  camera_info.K[6] = camera_matrix.at(6);
  camera_info.K[7] = camera_matrix.at(7);
  camera_info.K[8] = camera_matrix.at(8);
  camera_info.P[0] = projection_matrix.at(0);
  camera_info.P[1] = projection_matrix.at(1);
  camera_info.P[2] = projection_matrix.at(2);
  camera_info.P[3] = projection_matrix.at(3);
  camera_info.P[4] = projection_matrix.at(4);
  camera_info.P[5] = projection_matrix.at(5);
  camera_info.P[6] = projection_matrix.at(6);
  camera_info.P[7] = projection_matrix.at(7);
  camera_info.P[8] = projection_matrix.at(8);
  camera_info.P[9] = projection_matrix.at(9);
  camera_info.P[10] = projection_matrix.at(10);
  camera_info.P[11] = projection_matrix.at(11);

  ros::Publisher camera_info_publisher = pnh.advertise<sensor_msgs::CameraInfo>("output/camera_info", 1, false);

  ros::Rate rate(10);
  while (ros::ok())
  {
    camera_info_publisher.publish(camera_info);
    rate.sleep();
  }
  return 0;
}