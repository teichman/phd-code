#include <jarvis/detection_visualizer.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <openni2_interface/openni_helpers.h>
#include <asp/asp.h>
#include <bag_of_tricks/connected_components.h>

using namespace std;
using namespace asp;

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

  cv::imshow("color", color_vis_);
  cv::imshow("depth", depth_vis_);
  cv::waitKey(2);
  
  hrt_.start();
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
  //tracker_.update(msg);
}

