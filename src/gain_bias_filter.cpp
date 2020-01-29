#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class GainBiasFilter
{
private:
  ros::NodeHandle pnh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  double gain_;
  double bias_;

  void onImage(const sensor_msgs::ImageConstPtr& msg);
  void applyFilter(cv::Mat& image, const double gain, const double bias);
public:
  GainBiasFilter();
};

GainBiasFilter::GainBiasFilter() : pnh_("~"), it_(pnh_)
{
  // Subscrive to input video feed and publish output video feed
  image_sub_ = it_.subscribe("input/image", 1, &GainBiasFilter::onImage, this);
  image_pub_ = it_.advertise("output/image_filtered", 1);
  pnh_.param("gain", gain_, 0.0);
  pnh_.param("bias", bias_, 0.0);
}

void GainBiasFilter::applyFilter(cv::Mat& image,const double gain,const double bias)
{
  image *= gain;
  image += bias;
}

void GainBiasFilter::onImage(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  applyFilter(cv_ptr->image, gain_, bias_);

  // Output modified video stream
  image_pub_.publish(cv_ptr->toImageMsg());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gain_bias_filter");
  GainBiasFilter filter;
  ros::spin();
  return 0;
}
