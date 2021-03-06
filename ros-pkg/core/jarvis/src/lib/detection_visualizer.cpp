#include <jarvis/detection_visualizer.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <openni2_interface/openni_helpers.h>
#include <bag_of_tricks/connected_components.h>

using namespace std;

DetectionVisualizer::DetectionVisualizer(int width, int height) :
  tracker_(100)
{
  fg_sub_ = nh_.subscribe("foreground", 3, &DetectionVisualizer::foregroundCallback, this);
  bg_sub_ = nh_.subscribe("background", 3, &DetectionVisualizer::backgroundCallback, this);
  tracker_.visualize_ = true;
}

void DetectionVisualizer::backgroundCallback(sentinel::BackgroundConstPtr msg)
{
  reconstructor_.update(msg);
  if(reconstructor_.img_.rows > 0) {
    cv::imshow("background", reconstructor_.img_);
    cv::waitKey(2);
  }
}

void DetectionVisualizer::foregroundCallback(sentinel::ForegroundConstPtr msg)
{
  reconstructor_.update(msg);
  tracker_.update(msg);
  
  // -- Initialize data.
  if(color_vis_.rows != msg->height) {
    color_vis_ = cv::Mat3b(cv::Size(msg->width, msg->height));
    depth_vis_ = cv::Mat3b(cv::Size(msg->width, msg->height));
  }
  color_vis_ = cv::Vec3b(127, 127, 127);
  depth_vis_ = cv::Vec3b(0, 0, 0);
  
  // -- Draw tracks.
  tracker_.draw(color_vis_);
  cv::Mat3b color_vis_scaled;
  cv::resize(color_vis_, color_vis_scaled, color_vis_.size() * 2, cv::INTER_NEAREST);
  cv::imshow("tracks", color_vis_scaled);
  
  // -- Draw a depth visualization.
  depth_vis_ = cv::Vec3b(0, 0, 0);
  for(size_t i = 0; i < msg->indices.size(); ++i) {
    uint32_t idx = msg->indices[i];
    int y = idx / depth_vis_.cols;
    int x = idx - y * depth_vis_.cols;
    depth_vis_(y, x) = colorize(msg->depth[i] * 0.001, 0, 5);
  }
  for(size_t i = 0; i < msg->fg_indices.size(); ++i) {
    uint32_t idx = msg->fg_indices[i];
    int y = idx / msg->width;
    int x = idx - y * msg->width;
    cv::circle(depth_vis_, cv::Point(x, y), 2, cv::Scalar(0, 0, 255), -1);
  }
  for(size_t i = 0; i < msg->bg_fringe_indices.size(); ++i) {
    uint32_t idx = msg->bg_fringe_indices[i];
    int y = idx / msg->width;
    int x = idx - y * msg->width;
    cv::circle(depth_vis_, cv::Point(x, y), 2, cv::Scalar(0, 255, 0), -1);
  }

  cv::Mat3b depth_vis_scaled;
  cv::resize(depth_vis_, depth_vis_scaled, depth_vis_.size() * 2, cv::INTER_NEAREST);
  cv::imshow("depth", depth_vis_scaled);
  cv::waitKey(2);
}

