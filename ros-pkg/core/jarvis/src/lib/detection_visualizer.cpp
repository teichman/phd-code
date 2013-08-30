#include <jarvis/detection_visualizer.h>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;

DetectionVisualizer::DetectionVisualizer(int width, int height)
{
  sub_ = nh_.subscribe("detections", 1000, &DetectionVisualizer::callback, this);
                       //ros::TransportHints().unreliable().maxDatagramSize(100).tcpNoDelay());
  vis_ = cv::Mat3b(cv::Size(width, height), cv::Vec3b(0, 0, 0));
  cv::imshow("detection", vis_);
}

void DetectionVisualizer::callback(const sentinel::Detection& msg)
{
  cout << "Got a detection with " << msg.indices.size() << " points." << endl;

  ROS_ASSERT(msg.indices.size() == msg.depth.size());
  ROS_ASSERT(msg.color.size() == msg.depth.size() * 3);
  ROS_ASSERT((int)msg.height == vis_.rows);
  
  vis_ = cv::Vec3b(0, 0, 0);
  for(size_t i = 0; i < msg.indices.size(); ++i) {
    uint32_t idx = msg.indices[i];
    int y = idx / vis_.cols;
    int x = vis_.cols - 1 - (idx - y * vis_.cols);
    vis_(y, x)[0] = msg.color[i*3+2];
    vis_(y, x)[1] = msg.color[i*3+1];
    vis_(y, x)[2] = msg.color[i*3+0];
  }

  for(size_t i = 0; i < msg.fg_indices.size(); ++i) {
    uint32_t idx = msg.fg_indices[i];
    int y = idx / msg.width;
    int x = msg.width - 1 - (idx - y * msg.width);
    cv::circle(vis_, cv::Point(x, y), 2, cv::Scalar(0, 255, 0), -1);
  }
  for(size_t i = 0; i < msg.bg_fringe_indices.size(); ++i) {
    uint32_t idx = msg.bg_fringe_indices[i];
    int y = idx / msg.width;
    int x = msg.width - 1 - (idx - y * msg.width);
    cv::circle(vis_, cv::Point(x, y), 2, cv::Scalar(0, 0, 255), -1);
  }

  cv::Mat3b scaled;
  cv::resize(vis_, scaled, cv::Size(640, 480), cv::INTER_NEAREST);
  cv::imshow("detection", scaled);
  cv::waitKey(2);
}

