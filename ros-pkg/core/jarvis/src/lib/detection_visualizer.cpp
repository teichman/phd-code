#include <jarvis/detection_visualizer.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <openni2_interface/openni_helpers.h>
#include <bag_of_tricks/connected_components.h>

using namespace std;

// class DepthEPG : public EdgePotentialGenerator
// {
// public:
//   DECLARE_POD(DepthEPG);
  
//   DepthEPG(std::string name) :
//     EdgePotentialGenerator(name)
//   {
//     declareInput<cv::Mat1f>("Depth");  // meters
//     declareParam<double>("Sigma", 0.3);
//   }

// protected:
//   void compute();
//   void debug() const { writeEdgePotentialVisualization(); }
// };

// void DepthEPG::compute()
// {
//   cv::Mat1f depth = pull<cv::Mat1f>("Depth");
//   const SparseMat& structure = *pull<const SparseMat*>("EdgeStructure");
//   double sigma = param<double>("Sigma");
//   initializeStorage();

//   int idx = 0;
//   for(int y = 0; y < depth.rows; ++y) {
//     for(int x = 0; x < depth.cols; ++x, ++idx) {
//       float d0 = depth(y, x);
//       SparseMat::InnerIterator it(structure, idx);
//       for(; it; ++it) {
//         int y1, x1;
//         pixel(it.col(), depth.cols, &y1, &x1);
//         float d1 = depth(y1, x1);
//         edge_.insert(it.row(), it.col()) = exp(-fabs(d0 - d1) / sigma);
//       }
//     }
//   }

//   push<const SparseMat*>("Edge", &edge_);
// }

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
    color_vis_ = cv::Mat3b(cv::Size(msg->width, msg->height), cv::Vec3b(0, 0, 0));
    depth_vis_ = cv::Mat3b(cv::Size(msg->width, msg->height), cv::Vec3b(0, 0, 0));
  }
  
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

  // -- Make a visualization using the color image and foreground.
  // TODO: Make this use the tracking info.
  color_vis_ = cv::Vec3b(127, 127, 127);
  for(size_t i = 0; i < msg->indices.size(); ++i) {
    uint32_t idx = msg->indices[i];
    int y = idx / color_vis_.cols;
    int x = idx - y * color_vis_.cols;
    if(tracker_.foreground_(y, x) == 255) {
      color_vis_(y, x)[0] = msg->color[i*3+2];
      color_vis_(y, x)[1] = msg->color[i*3+1];
      color_vis_(y, x)[2] = msg->color[i*3+0];
    }
  }

  cv::Mat3b color_vis_scaled;
  cv::resize(color_vis_, color_vis_scaled, color_vis_.size() * 2, cv::INTER_NEAREST);
  cv::imshow("color", color_vis_scaled);
  cv::waitKey(2);
}

