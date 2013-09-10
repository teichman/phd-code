#include <jarvis/detection_visualizer.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <openni2_interface/openni_helpers.h>
#include <asp/asp.h>
#include <bag_of_tricks/connected_components.h>

using namespace std;
using namespace asp;

void flood(cv::Mat1f depth, float thresh, int min_pts, const cv::Point2i& seed, int id, cv::Mat1i* ass)
{
  vector<cv::Point2i> pts;
  pts.reserve(min_pts);
  pts.push_back(seed);
  
  std::queue<cv::Point2i> que;
  int num_pts = 1;
  que.push(seed);
  (*ass)(seed) = -2;
  while(!que.empty()) {
    cv::Point2i pt = que.front();
    que.pop();

    ROS_ASSERT((*ass)(pt) == -2);
    (*ass)(pt) = id;
    float d0 = depth(pt);
    
    cv::Point2i dpt;
    for(dpt.y = -1; dpt.y <= 1; ++dpt.y) {
      for(dpt.x = -1; dpt.x <= 1; ++dpt.x) {
        cv::Point2i pt2 = pt + dpt;
        if(pt2.x < 0 || pt2.x >= ass->cols ||
           pt2.y < 0 || pt2.y >= ass->rows)
          continue;
        float d2 = depth(pt2);

        if(fabs(d0 - d2) < thresh && (*ass)(pt2) == -3) {
          que.push(pt2);
          (*ass)(pt2) = -2;
          ++num_pts;
          if(num_pts < min_pts)
            pts.push_back(pt2);
        }
      }
    }
  }

  // If we didn't get enough points in this cluster, backtrack and remove them.
  if(num_pts < min_pts)
    for(size_t i = 0; i < pts.size(); ++i)
      (*ass)(pts[i]) = -1;
}

//! depth has 0 wherever there is nothing and a depth value where there is something to be clustered.
//! assignments will be filled with -1s for no object and with the object id otherwise.
void cluster(cv::Mat1f depth, float thresh, int min_pts, cv::Mat1i* assignments)
{
  // -3    : unset
  // -2    : in queue
  // -1    : bg
  // >= 0    : cluster assignment
  ROS_ASSERT(depth.size() == assignments->size());
  *assignments = -3;
  for(int y = 0; y < assignments->rows; ++y)
    for(int x = 0; x < assignments->cols; ++x)
      if(depth(y, x) == 0)
        (*assignments)(y, x) = -1;

  int id = 0;
  for(int y = 0; y < assignments->rows; ++y) {
    for(int x = 0; x < assignments->cols; ++x) {
      if((*assignments)(y, x) == -3) {
        cv::KeyPoint center;
        cv::Point2i seed(x, y);
        flood(depth, thresh, min_pts, seed, id, assignments);
        ++id;
      }
    }
  }
}

class DepthEPG : public EdgePotentialGenerator
{
public:
  DECLARE_POD(DepthEPG);
  
  DepthEPG(std::string name) :
    EdgePotentialGenerator(name)
  {
    declareInput<cv::Mat1f>("Depth");  // meters
    declareParam<double>("Sigma", 0.3);
  }

protected:
  void compute();
  void debug() const { writeEdgePotentialVisualization(); }
};

void DepthEPG::compute()
{
  cv::Mat1f depth = pull<cv::Mat1f>("Depth");
  const SparseMat& structure = *pull<const SparseMat*>("EdgeStructure");
  double sigma = param<double>("Sigma");
  initializeStorage();

  int idx = 0;
  for(int y = 0; y < depth.rows; ++y) {
    for(int x = 0; x < depth.cols; ++x, ++idx) {
      float d0 = depth(y, x);
      SparseMat::InnerIterator it(structure, idx);
      for(; it; ++it) {
        int y1, x1;
        pixel(it.col(), depth.cols, &y1, &x1);
        float d1 = depth(y1, x1);
        edge_.insert(it.row(), it.col()) = exp(-fabs(d0 - d1) / sigma);
      }
    }
  }

  push<const SparseMat*>("Edge", &edge_);
}

DetectionVisualizer::DetectionVisualizer(int width, int height) :
  asp_(4)
{
  fg_sub_ = nh_.subscribe("foreground", 3, &DetectionVisualizer::foregroundCallback, this);
  bg_sub_ = nh_.subscribe("background", 3, &DetectionVisualizer::backgroundCallback, this);

  color_vis_ = cv::Mat3b(cv::Size(width, height), cv::Vec3b(0, 0, 0));
  depth_vis_ = cv::Mat3b(cv::Size(width, height), cv::Vec3b(0, 0, 0));
  cv::imshow("color", color_vis_);
  cv::imshow("depth", depth_vis_);
  cv::waitKey(2);
  
  hrt_.start();
}

void DetectionVisualizer::backgroundCallback(const sentinel::Background& msg)
{
  if(background_.rows != msg.height)
    background_ = cv::Mat3b(cv::Size(msg.width, msg.height), cv::Vec3b(0, 0, 0));

  for(size_t i = 0; i < msg.indices.size(); ++i) {
    int idx = msg.indices[i];
    int y = idx / background_.cols;
    int x = idx - y * background_.cols;
    ROS_ASSERT(y >= 0 && y < background_.rows);
    ROS_ASSERT(x >= 0 && x < background_.cols);
    background_(y, x)[0] = msg.color[i*3+2];
    background_(y, x)[1] = msg.color[i*3+1];
    background_(y, x)[2] = msg.color[i*3+0];
  }

  cv::imshow("background", background_);
  cv::waitKey(2);
}

void DetectionVisualizer::foregroundCallback(const sentinel::Foreground& msg)
{
  HighResTimer hrt;
  
  // -- Debugging.
  // cout << "Got a detection with " << msg.indices.size() << " points." << endl;
  // cout << msg.depth.size() << " " << msg.color.size() << endl;

  ROS_ASSERT(msg.indices.size() == msg.depth.size());
  ROS_ASSERT(msg.color.size() == msg.depth.size() * 3);
  ROS_ASSERT((int)msg.height == color_vis_.rows);
  ROS_ASSERT(msg.width % msg.width_step == 0);
  ROS_ASSERT(msg.height % msg.height_step == 0);
  ROS_ASSERT(msg.width == 320 && msg.height == 240);

  hrt.reset("bilateral filter");
  hrt.start();
  
  cv::Mat1f depth(color_vis_.size(), 0);
  for(size_t i = 0; i < msg.indices.size(); ++i) {
    uint32_t idx = msg.indices[i];
    int y = idx / depth.cols;
    int x = idx - y * depth.cols;
    depth(y, x) = msg.depth[i] * 0.001;
  }

  depth_vis_ = cv::Vec3b(0, 0, 0);
  for(size_t i = 0; i < msg.indices.size(); ++i) {
    uint32_t idx = msg.indices[i];
    int y = idx / depth_vis_.cols;
    int x = idx - y * depth_vis_.cols;
    depth_vis_(y, x) = colorize(msg.depth[i] * 0.001, 0, 5);
  }
  for(size_t i = 0; i < msg.fg_indices.size(); ++i) {
    uint32_t idx = msg.fg_indices[i];
    int y = idx / msg.width;
    int x = idx - y * msg.width;
    cv::circle(depth_vis_, cv::Point(x, y), 2, cv::Scalar(0, 0, 255), -1);
  }
  for(size_t i = 0; i < msg.bg_fringe_indices.size(); ++i) {
    uint32_t idx = msg.bg_fringe_indices[i];
    int y = idx / msg.width;
    int x = idx - y * msg.width;
    cv::circle(depth_vis_, cv::Point(x, y), 2, cv::Scalar(0, 255, 0), -1);
  }

  
  cv::Mat1b indices_mask(cv::Size(msg.width, msg.height), 0);
  for(size_t i = 0; i < msg.indices.size(); ++i) {
    uint32_t idx = msg.indices[i];
    int y = idx / depth_vis_.cols;
    int x = idx - y * depth_vis_.cols;
    indices_mask(y, x) = 255;
  }
    
  cv::Mat1f values(cv::Size(msg.width, msg.height), 0);
  double sigma_p = 10;
  double sigma_d = 0.2;

  for(size_t i = 0; i < msg.fg_indices.size(); ++i) {
    uint32_t idx = msg.fg_indices[i];
    int y = idx / msg.width;
    int x = idx - y * msg.width;
    int dx = msg.width_step / 2 + 1;
    int dy = msg.height_step / 2 + 1;
    for(int y2 = y - dy; y2 <= y + dy; ++y2) {
      for(int x2 = x - dx; x2 <= x + dx; ++x2) {
        if(y2 < 0 || y2 >= msg.height ||
           x2 < 0 || x2 >= msg.width)
        {
          continue;
        }
        double img_coef = exp(-sqrt((x - x2)*(x - x2) + (y - y2)*(y - y2)) / sigma_p);
        double depth_coef = exp(-fabs(depth(y, x) - depth(y2, x2)) / sigma_d);
        values(y2, x2) += img_coef * depth_coef;
      }
    }
  }
  for(size_t i = 0; i < msg.bg_fringe_indices.size(); ++i) {
    uint32_t idx = msg.bg_fringe_indices[i];
    int y = idx / msg.width;
    int x = idx - y * msg.width;
    int dx = msg.width_step / 2 + 1;
    int dy = msg.height_step / 2 + 1;
    for(int y2 = y - dy; y2 <= y + dy; ++y2) {
      for(int x2 = x - dx; x2 <= x + dx; ++x2) {
        if(y2 < 0 || y2 >= msg.height ||
           x2 < 0 || x2 >= msg.width)
        {
          continue;
        }
        double img_coef = exp(-sqrt((x - x2)*(x - x2) + (y - y2)*(y - y2)) / sigma_p);
        double depth_coef = exp(-fabs(depth(y, x) - depth(y2, x2)) / sigma_d);
        values(y2, x2) -= img_coef * depth_coef;
      }
    }
  }

  cv::Mat1b foreground(cv::Size(msg.width, msg.height), 0);
  for(int y = 0; y < values.rows; ++y)
    for(int x = 0; x < values.cols; ++x)
      if(values(y, x) > 0 && indices_mask(y, x) == 255 && depth(y, x) > 1e-3)
        foreground(y, x) = 255;

  hrt.stop(); cout << hrt.reportMilliseconds() << endl;
  
  cv::Mat3b vis(foreground.size(), cv::Vec3b(0, 0, 0));
  for(int y = 0; y < vis.rows; ++y)
    for(int x = 0; x < vis.cols; ++x)
      if(foreground(y, x) == 255)
        vis(y, x) = cv::Vec3b(255, 255, 255);

  for(size_t i = 0; i < msg.fg_indices.size(); ++i) {
    uint32_t idx = msg.fg_indices[i];
    int y = idx / msg.width;
    int x = idx - y * msg.width;
    cv::circle(vis, cv::Point(x, y), 2, cv::Scalar(0, 0, 255), -1);
  }
  for(size_t i = 0; i < msg.bg_fringe_indices.size(); ++i) {
    uint32_t idx = msg.bg_fringe_indices[i];
    int y = idx / msg.width;
    int x = idx - y * msg.width;
    cv::circle(vis, cv::Point(x, y), 2, cv::Scalar(0, 255, 0), -1);
  }

  // cv::Mat3b vis_scaled;
  // cv::resize(vis, vis_scaled, vis.size() * 2, cv::INTER_NEAREST);
  // cv::imshow("foreground visualization", vis_scaled);

  // cv::Mat1b foreground_scaled;
  // cv::resize(foreground, foreground_scaled, foreground.size() * 2, cv::INTER_NEAREST);
  // cv::imshow("foreground", foreground_scaled);

  cv::Mat3b depth_vis_scaled;
  cv::resize(depth_vis_, depth_vis_scaled, depth_vis_.size() * 2, cv::INTER_NEAREST);
  cv::imshow("depth", depth_vis_scaled);

  cv::Mat1i blobs(cv::Size(msg.width, msg.height), 0);
  for(int y = 0; y < depth.rows; ++y)
    for(int x = 0; x < depth.cols; ++x)
      if(foreground(y, x) == 0)
        depth(y, x) = 0;

  hrt.reset("clustering"); hrt.start();
  cluster(depth, 0.2, 400, &blobs);
  hrt.stop(); cout << hrt.reportMilliseconds() << endl;
  cv::Mat3b clustering_scaled;
  cv::resize(colorAssignments(blobs), clustering_scaled, blobs.size() * 2, cv::INTER_NEAREST);
  cv::imshow("Clustering", clustering_scaled);

  // -- The foreground should only contain points in large-enough clusters.
  for(int y = 0; y < blobs.rows; ++y)
    for(int x = 0; x < blobs.cols; ++x)
      if(blobs(y, x) < 0)
        foreground(y, x) = 0;

  // -- Make a visualization using the color image and foreground.
  color_vis_ = cv::Vec3b(127, 127, 127);
  for(size_t i = 0; i < msg.indices.size(); ++i) {
    uint32_t idx = msg.indices[i];
    int y = idx / color_vis_.cols;
    int x = idx - y * color_vis_.cols;
    if(foreground(y, x) == 255) {
      color_vis_(y, x)[0] = msg.color[i*3+2];
      color_vis_(y, x)[1] = msg.color[i*3+1];
      color_vis_(y, x)[2] = msg.color[i*3+0];
    }
  }

  cv::Mat3b color_vis_scaled;
  cv::resize(color_vis_, color_vis_scaled, color_vis_.size() * 2, cv::INTER_NEAREST);
  cv::imshow("color", color_vis_scaled);
  cv::waitKey(2);
}

