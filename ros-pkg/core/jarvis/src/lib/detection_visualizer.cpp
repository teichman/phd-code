#include <jarvis/detection_visualizer.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <openni2_interface/openni_helpers.h>

using namespace std;

DetectionVisualizer::DetectionVisualizer(int width, int height)
{
  sub_ = nh_.subscribe("detections", 1000, &DetectionVisualizer::callback, this);
                       //ros::TransportHints().unreliable().maxDatagramSize(100).tcpNoDelay());
  color_vis_ = cv::Mat3b(cv::Size(width, height), cv::Vec3b(0, 0, 0));
  depth_vis_ = cv::Mat3b(cv::Size(width, height), cv::Vec3b(0, 0, 0));
  cv::imshow("color", color_vis_);
  cv::imshow("depth", depth_vis_);
  cv::waitKey(2);
}

void DetectionVisualizer::callback(const sentinel::Detection& msg)
{
  cout << "Got a detection with " << msg.indices.size() << " points." << endl;

  ROS_ASSERT(msg.indices.size() == msg.depth.size());
  ROS_ASSERT(msg.color.size() == msg.depth.size() * 3);
  ROS_ASSERT((int)msg.height == color_vis_.rows);

  depth_vis_ = cv::Vec3b(0, 0, 0);
  for(size_t i = 0; i < msg.indices.size(); ++i) {
    uint32_t idx = msg.indices[i];
    int y = idx / depth_vis_.cols;
    int x = depth_vis_.cols - 1 - (idx - y * depth_vis_.cols);
    depth_vis_(y, x) = colorize(msg.depth[i] * 0.001, 0, 10);
  }
  
  color_vis_ = cv::Vec3b(0, 0, 0);
  for(size_t i = 0; i < msg.indices.size(); ++i) {
    uint32_t idx = msg.indices[i];
    int y = idx / color_vis_.cols;
    int x = color_vis_.cols - 1 - (idx - y * color_vis_.cols);
    color_vis_(y, x)[0] = msg.color[i*3+2];
    color_vis_(y, x)[1] = msg.color[i*3+1];
    color_vis_(y, x)[2] = msg.color[i*3+0];
  }

  for(size_t i = 0; i < msg.fg_indices.size(); ++i) {
    uint32_t idx = msg.fg_indices[i];
    int y = idx / msg.width;
    int x = msg.width - 1 - (idx - y * msg.width);
    cv::circle(color_vis_, cv::Point(x, y), 2, cv::Scalar(0, 255, 0), -1);
  }
  for(size_t i = 0; i < msg.bg_fringe_indices.size(); ++i) {
    uint32_t idx = msg.bg_fringe_indices[i];
    int y = idx / msg.width;
    int x = msg.width - 1 - (idx - y * msg.width);
    cv::circle(color_vis_, cv::Point(x, y), 2, cv::Scalar(0, 0, 255), -1);
  }

  cv::Mat3b depth_scaled;
  cv::resize(depth_vis_, depth_scaled, cv::Size(640, 480), cv::INTER_NEAREST);
  cv::imshow("depth", depth_scaled);
  cv::Mat3b color_scaled;
  cv::resize(color_vis_, color_scaled, cv::Size(640, 480), cv::INTER_NEAREST);
  cv::imshow("color", color_scaled);
  cv::waitKey(2);
}

