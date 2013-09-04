#include <jarvis/detection_visualizer.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <openni2_interface/openni_helpers.h>
#include <asp/asp.h>

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
  sub_ = nh_.subscribe("detections", 1000, &DetectionVisualizer::callback, this);
                       //ros::TransportHints().unreliable().maxDatagramSize(100).tcpNoDelay());
  color_vis_ = cv::Mat3b(cv::Size(width, height), cv::Vec3b(0, 0, 0));
  depth_vis_ = cv::Mat3b(cv::Size(width, height), cv::Vec3b(0, 0, 0));
  cv::imshow("color", color_vis_);
  cv::imshow("depth", depth_vis_);
  cv::waitKey(2);

  asp_.addPod(new EntryPoint<cv::Mat1f>("DepthEntryPoint"));
  asp_.addPod(new EdgeStructureGenerator("EdgeStructureGenerator"));
  asp_.setParam("EdgeStructureGenerator", "Grid", true);
  asp_.setParam("EdgeStructureGenerator", "Diagonal", true);
  asp_.connect("ImageEntryPoint.Output -> EdgeStructureGenerator.Image");
  asp_.connect("MaskEntryPoint.Output -> EdgeStructureGenerator.Mask");
  asp_.addPod(new DepthEPG("DepthEPG"));
  asp_.connect("ImageEntryPoint.Output -> DepthEPG.Image");
  asp_.connect("DepthEntryPoint.Output -> DepthEPG.Depth");
  asp_.connect("EdgeStructureGenerator.EdgeStructure -> DepthEPG.EdgeStructure");
  asp_.connect("DepthEPG.Edge -> EdgePotentialAggregator.UnweightedEdge");
  gc::Model model = asp_.defaultModel();
  model.nweights_(model.nameMapping("nmap").toId("PriorNPG")) = 0.001;
  asp_.setModel(model);
  cout << model << endl;
  asp_.writeGraphviz("graphvis");
  
  hrt_.start();
}

void DetectionVisualizer::callback(const sentinel::Detection& msg)
{
  // -- Debugging.
  cout << "Got a detection with " << msg.indices.size() << " points." << endl;
  cout << msg.depth.size() << " " << msg.color.size() << endl;

  ROS_ASSERT(msg.indices.size() == msg.depth.size());
  ROS_ASSERT(msg.color.size() == msg.depth.size() * 3);
  ROS_ASSERT((int)msg.height == color_vis_.rows);
  
  // -- Output a framerate estimate.
  timestamps_.push_back(hrt_.getSeconds());
  if(timestamps_.size() > 100)
    timestamps_.pop_front();
  if(timestamps_.size() == 100)
    cout << "FPS: " << timestamps_.size() / (timestamps_.back() - timestamps_.front()) << endl;

  cv::Mat1b mask(color_vis_.size(), 0);
  for(size_t i = 0; i < msg.indices.size(); ++i) {
    uint32_t idx = msg.indices[i];
    int y = idx / depth_vis_.cols;
    int x = depth_vis_.cols - 1 - (idx - y * depth_vis_.cols);
    mask(y, x) = 255;
  }
  
  // -- Make depth visualization.
  depth_vis_ = cv::Vec3b(0, 0, 0);
  for(size_t i = 0; i < msg.indices.size(); ++i) {
    uint32_t idx = msg.indices[i];
    int y = idx / depth_vis_.cols;
    int x = depth_vis_.cols - 1 - (idx - y * depth_vis_.cols);
    depth_vis_(y, x) = colorize(msg.depth[i] * 0.001, 0, 10);
  }

  // -- Make color visualization.
  color_vis_ = cv::Vec3b(0, 0, 0);
  for(size_t i = 0; i < msg.indices.size(); ++i) {
    uint32_t idx = msg.indices[i];
    int y = idx / color_vis_.cols;
    int x = color_vis_.cols - 1 - (idx - y * color_vis_.cols);
    color_vis_(y, x)[0] = msg.color[i*3+2];
    color_vis_(y, x)[1] = msg.color[i*3+1];
    color_vis_(y, x)[2] = msg.color[i*3+0];
  }

  // -- Add foreground and background markers.
  //    Construct seed image while at it.
  cv::Mat1b seed(color_vis_.size(), 127);
  for(size_t i = 0; i < msg.fg_indices.size(); ++i) {
    uint32_t idx = msg.fg_indices[i];
    int y = idx / msg.width;
    int x = msg.width - 1 - (idx - y * msg.width);
    //cv::circle(color_vis_, cv::Point(x, y), 2, cv::Scalar(0, 255, 0), -1);
    seed(y, x) = 255;
  }
  for(size_t i = 0; i < msg.bg_fringe_indices.size(); ++i) {
    uint32_t idx = msg.bg_fringe_indices[i];
    int y = idx / msg.width;
    int x = msg.width - 1 - (idx - y * msg.width);
    //cv::circle(color_vis_, cv::Point(x, y), 2, cv::Scalar(0, 0, 255), -1);
    seed(y, x) = 0;
  }

  // -- Display.
  cv::Mat3b depth_scaled;
  cv::resize(depth_vis_, depth_scaled, cv::Size(640, 480), cv::INTER_NEAREST);
  cv::imshow("depth", depth_scaled);
  cv::Mat3b color_scaled;
  cv::resize(color_vis_, color_scaled, cv::Size(640, 480), cv::INTER_NEAREST);
  cv::imshow("color", color_scaled);

  // -- Compute a more detailed foreground / background mask.
  //    Will be 255 for foreground points.
  asp_.setInput("ImageEntryPoint", color_vis_);
  asp_.setInput("MaskEntryPoint", mask);
  asp_.setInput("SeedEntryPoint", seed);
  
  // Make depth image for use in segmentation.
  cv::Mat1f depth(color_vis_.size(), 0);
  for(size_t i = 0; i < msg.indices.size(); ++i) {
    uint32_t idx = msg.indices[i];
    int y = idx / depth.cols;
    int x = depth.cols - 1 - (idx - y * depth.cols);
    depth(y, x) = msg.depth[i] * 0.001;
  }
  asp_.setInput("DepthEntryPoint", depth);
  //asp_.setDebug(true);
  cv::Mat1b segmentation(cv::Size(msg.width, msg.height), 0);  
  asp_.segment(&segmentation);
  cout << asp_.reportTiming() << endl;
  
  cv::imshow("Segmentation", segmentation);
  cv::waitKey(5);

  // -- Run connected components on that mask.
  // ROS_ASSERT(msg.width % msg.width_step == 0 && msg.height % msg.height_step == 0);
  // cv::Mat1b fg(cv::Size(msg.width / msg.width_step, msg.height / msg.height_step), 0);
}

