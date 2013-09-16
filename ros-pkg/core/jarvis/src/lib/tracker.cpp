#include <jarvis/tracker.h>
#include <bag_of_tricks/connected_components.h>
#include <sentinel/background_model.h>


using namespace std;


Tracker::Tracker(size_t max_track_length) :
  visualize_(false),
  max_track_length_(max_track_length)
{
}

void Tracker::update(sentinel::ForegroundConstPtr msg)
{
  HighResTimer hrt;

  // -- Debugging.
  // cout << "Got a detection with " << msg->indices.size() << " points." << endl;
  // cout << msg->depth.size() << " " << msg->color.size() << endl;

  // -- Allocate memory if necessary.
  if(depth_.rows != msg->height) {
    depth_ = cv::Mat1f(cv::Size(msg->width, msg->height), 0);
    foreground_ = cv::Mat1b(cv::Size(msg->width, msg->height), 0);
    blobs_ = cv::Mat1i(cv::Size(msg->width, msg->height), 0);
  }

  // -- Check the data.
  ROS_ASSERT(msg->indices.size() == msg->depth.size());
  ROS_ASSERT(msg->color.size() == msg->depth.size() * 3);
  ROS_ASSERT((int)msg->height == depth_.rows);
  ROS_ASSERT(msg->width % msg->width_step == 0);
  ROS_ASSERT(msg->height % msg->height_step == 0);
  
  // -- Build the depth image.
  depth_ = 0;
  for(size_t i = 0; i < msg->indices.size(); ++i) {
    uint32_t idx = msg->indices[i];
    int y = idx / depth_.cols;
    int x = idx - y * depth_.cols;
    depth_(y, x) = msg->depth[i] * 0.001;
  }

  // -- Run the bilateral filter to get the detailed foreground mask.
  reconstructForeground(msg, depth_, foreground_);

  // -- Floodfill clustering.
  //    Ignore parts of the depth image that were not part of the foreground.
  for(int y = 0; y < depth_.rows; ++y)
    for(int x = 0; x < depth_.cols; ++x)
      if(foreground_(y, x) == 0)
        depth_(y, x) = 0;

  // Run the clustering.
  hrt.reset("clustering"); hrt.start();
  cluster(depth_, 0.2, 1, &blobs_);
  hrt.stop(); cout << hrt.reportMilliseconds() << endl;
  if(visualize_) { 
    cv::Mat3b clustering_scaled;
    cv::resize(colorAssignments(blobs_), clustering_scaled, blobs_.size() * 2, cv::INTER_NEAREST);
    cv::imshow("Clustering", clustering_scaled);
  }

  // -- The foreground should only contain points in large-enough clusters.
  for(int y = 0; y < blobs_.rows; ++y)
    for(int x = 0; x < blobs_.cols; ++x)
      if(blobs_(y, x) < 0)
        foreground_(y, x) = 0;

  // -- Simple correspondence.

  // -- Update tracks.
}

void Tracker::reconstructForeground(sentinel::Foreground::ConstPtr msg,
                                    cv::Mat1f depth, cv::Mat1b foreground) const
{
  ScopedTimer st("Tracker::reconstructForeground", TimeUnit::MS);
  
  cv::Mat1b indices_mask(cv::Size(msg->width, msg->height), 0);
  for(size_t i = 0; i < msg->indices.size(); ++i) {
    uint32_t idx = msg->indices[i];
    int y = idx / depth_.cols;
    int x = idx - y * depth_.cols;
    indices_mask(y, x) = 255;
  }
    
  cv::Mat1f values(cv::Size(msg->width, msg->height), 0);
  double sigma_p = 10;
  double sigma_d = 0.2;

  for(size_t i = 0; i < msg->fg_indices.size(); ++i) {
    uint32_t idx = msg->fg_indices[i];
    int y = idx / msg->width;
    int x = idx - y * msg->width;
    int dx = msg->width_step / 2 + 1;
    int dy = msg->height_step / 2 + 1;
    for(int y2 = y - dy; y2 <= y + dy; ++y2) {
      for(int x2 = x - dx; x2 <= x + dx; ++x2) {
        if(y2 < 0 || y2 >= msg->height ||
           x2 < 0 || x2 >= msg->width)
        {
          continue;
        }
        double img_coef = exp(-sqrt((x - x2)*(x - x2) + (y - y2)*(y - y2)) / sigma_p);
        double depth_coef = exp(-fabs(depth(y, x) - depth(y2, x2)) / sigma_d);
        values(y2, x2) += img_coef * depth_coef;
      }
    }
  }
  for(size_t i = 0; i < msg->bg_fringe_indices.size(); ++i) {
    uint32_t idx = msg->bg_fringe_indices[i];
    int y = idx / msg->width;
    int x = idx - y * msg->width;
    int dx = msg->width_step / 2 + 1;
    int dy = msg->height_step / 2 + 1;
    for(int y2 = y - dy; y2 <= y + dy; ++y2) {
      for(int x2 = x - dx; x2 <= x + dx; ++x2) {
        if(y2 < 0 || y2 >= msg->height ||
           x2 < 0 || x2 >= msg->width)
        {
          continue;
        }
        double img_coef = exp(-sqrt((x - x2)*(x - x2) + (y - y2)*(y - y2)) / sigma_p);
        double depth_coef = exp(-fabs(depth(y, x) - depth(y2, x2)) / sigma_d);
        values(y2, x2) -= img_coef * depth_coef;
      }
    }
  }

  // -- Fill the output values.
  foreground = 0;
  for(int y = 0; y < values.rows; ++y)
    for(int x = 0; x < values.cols; ++x)
      if(values(y, x) > 0 && indices_mask(y, x) == 255 && depth(y, x) > 1e-3)
        foreground(y, x) = 255;
}
