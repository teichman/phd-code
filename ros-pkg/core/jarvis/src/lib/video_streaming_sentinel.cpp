#include <jarvis/video_streaming_sentinel.h>
#include <bag_of_tricks/next_path.h>

using namespace std;

VideoStreamingSentinel::VideoStreamingSentinel(double update_interval,
                                               double occupancy_threshold,
                                               int raytracing_threshold,
                                               double detection_threshold,
                                               bool visualize,
                                               OpenNI2Interface::Resolution color_res,
                                               OpenNI2Interface::Resolution depth_res) :
  Sentinel(update_interval, occupancy_threshold, raytracing_threshold,
           detection_threshold, visualize,
           color_res, depth_res),
  timeout_(0)
{
  images_.reserve(1000);
  serializer_.delay_ = 30;
  serializer_.launch();
}

VideoStreamingSentinel::~VideoStreamingSentinel()
{
  // Wait for the serializer to finish.
  serializer_.stop();
}

void VideoStreamingSentinel::handleDetection(openni::VideoFrameRef color,
                                             openni::VideoFrameRef depth,
                                             const std::vector<uint32_t>& indices,
                                             const std::vector<uint32_t>& fg_markers,
                                             const std::vector<uint32_t>& bg_fringe_markers,
                                             double sensor_timestamp,
                                             double wall_timestamp,
                                             size_t frame_id)
{
  // Make a visualization image and add to the video buffer.
  images_.push_back(visualize(color, depth));
  

  cv::Mat3b img = images_.back();
  float val = 0;
  for(int y = 0; y < img.rows; ++y)
    for(int x = 0; x < img.cols; ++x)
      val += img(y, x)[0];

  cout << "[VideoStreamingSentinel] Detection " << images_.size() << ".  Hash: " << val << endl;
  
  timeout_ = 0;
}

void VideoStreamingSentinel::handleNonDetection(openni::VideoFrameRef color,
                                                openni::VideoFrameRef depth,
                                                double sensor_timestamp,
                                                double wall_timestamp,
                                                size_t frame_id)
{
  // If we haven't seen a detection in a while, flush the buffer.
  ++timeout_;
  if(timeout_ > 60 && !images_.empty()) {
    cout << "[VideoStreamingSentinel] Flushing." << endl;
    serializer_.push(images_, nextPath(".", "", "-movement.avi", 4));
    images_.clear();
    timeout_ = 0;
  }
}

void VideoStreamingSentinel::processHook(openni::VideoFrameRef color, openni::VideoFrameRef depth, double wall_timestamp)
{
  // Do nothing.
}


