#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <vector>
#include <string>

class CameraInfoPublisher
{
private:
  ros::NodeHandle pnh_;
  // parameters
  std::vector<double> camera_matrix_;
  std::vector<double> distortion_coefficients_;
  std::vector<double> projection_matrix_;
  int image_width_;
  int image_height_;
  std::string frame_id_;
  std::string distortion_model_;
  ros::Publisher camera_info_publisher_;
  ros::Subscriber camera_image_subscriber_;
  void onImage(const sensor_msgs::ImageConstPtr& msg);

public:
  CameraInfoPublisher();
};

CameraInfoPublisher::CameraInfoPublisher() : pnh_("~")
{
  // receive parameters
  if (!pnh_.hasParam("camera_matrix/data"))
  {
    ROS_ERROR("missing camera_matrix!!");
    std::exit(1);
  }
  if (!pnh_.hasParam("distortion_coefficients/data"))
  {
    ROS_ERROR("missing distortion_coefficients!!");
    std::exit(1);
  }
  if (!pnh_.hasParam("projection_matrix/data"))
  {
    ROS_ERROR("missing projection_matrix!!");
    std::exit(1);
  }
  pnh_.param<std::vector<double>>("camera_matrix/data", camera_matrix_, camera_matrix_);
  pnh_.param<std::vector<double>>("distortion_coefficients/data", distortion_coefficients_, distortion_coefficients_);
  pnh_.param<std::vector<double>>("projection_matrix/data", projection_matrix_, projection_matrix_);
  pnh_.param<int>("image_height", image_height_, 0);
  pnh_.param<int>("image_width", image_width_, 0);
  pnh_.param<std::string>("distortion_model", distortion_model_, "plum_blob");

  camera_info_publisher_ = pnh_.advertise<sensor_msgs::CameraInfo>("output/camera_info", 1, false);
  camera_image_subscriber_ =
      pnh_.subscribe<sensor_msgs::Image>("input/image", 1, &CameraInfoPublisher::onImage, this);
}

void CameraInfoPublisher::onImage(const sensor_msgs::ImageConstPtr& msg)
{
  // create message
  sensor_msgs::CameraInfo camera_info;
  camera_info.header = msg->header;
  camera_info.height = image_height_;
  camera_info.width = image_width_;
  camera_info.distortion_model = distortion_model_;
  camera_info.D = distortion_coefficients_;
  camera_info.K[0] = camera_matrix_.at(0);
  camera_info.K[1] = camera_matrix_.at(1);
  camera_info.K[2] = camera_matrix_.at(2);
  camera_info.K[3] = camera_matrix_.at(3);
  camera_info.K[4] = camera_matrix_.at(4);
  camera_info.K[5] = camera_matrix_.at(5);
  camera_info.K[6] = camera_matrix_.at(6);
  camera_info.K[7] = camera_matrix_.at(7);
  camera_info.K[8] = camera_matrix_.at(8);
  camera_info.P[0] = projection_matrix_.at(0);
  camera_info.P[1] = projection_matrix_.at(1);
  camera_info.P[2] = projection_matrix_.at(2);
  camera_info.P[3] = projection_matrix_.at(3);
  camera_info.P[4] = projection_matrix_.at(4);
  camera_info.P[5] = projection_matrix_.at(5);
  camera_info.P[6] = projection_matrix_.at(6);
  camera_info.P[7] = projection_matrix_.at(7);
  camera_info.P[8] = projection_matrix_.at(8);
  camera_info.P[9] = projection_matrix_.at(9);
  camera_info.P[10] = projection_matrix_.at(10);
  camera_info.P[11] = projection_matrix_.at(11);

  camera_info_publisher_.publish(camera_info);
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "camera_info_publisher");
  CameraInfoPublisher cam_info_publisher;

  ros::spin();
  return 0;
}