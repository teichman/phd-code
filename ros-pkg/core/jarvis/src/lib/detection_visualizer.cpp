#include <jarvis/detection_visualizer.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <openni2_interface/openni_helpers.h>
#include <asp/asp.h>
#include <bag_of_tricks/connected_components.h>

using namespace std;
using namespace asp;

int g_scaling = 3;
int g_y = -1;
int g_x = -1;

void mouseEvent(int evt, int x, int y, int flags, void* param){
  if(evt==CV_EVENT_LBUTTONDOWN){
    g_x = x / g_scaling;
    g_y = y / g_scaling;
    cout << "Set debug point to " << g_x << " " << g_y << endl;
  }
}

void flood(cv::Mat1f depth, float thresh, const cv::Point2i& seed, int id, cv::Mat1i* ass)
{
  std::queue<cv::Point2i> que;
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
        }
      }
    }
  }
}

//! depth has 0 wherever there is nothing and a depth value where there is something to be clustered.
//! assignments will be filled with -1s for no object and with the object id otherwise.
void cluster(cv::Mat1f depth, float thresh, cv::Mat1i* assignments)
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
        flood(depth, thresh, seed, id, assignments);
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
  sub_ = nh_.subscribe("detections", 3, &DetectionVisualizer::callback, this);
                       //ros::TransportHints().unreliable().maxDatagramSize(100).tcpNoDelay());
  color_vis_ = cv::Mat3b(cv::Size(width, height), cv::Vec3b(0, 0, 0));
  depth_vis_ = cv::Mat3b(cv::Size(width, height), cv::Vec3b(0, 0, 0));
  cv::imshow("color", color_vis_);
  cv::imshow("depth", depth_vis_);
  cv::imshow("bilateral_foreground", depth_vis_);
  cv::waitKey(2);
  cv::setMouseCallback("bilateral_foreground", mouseEvent, 0);

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
  model.nweights_(model.nameMapping("nmap").toId("PriorNPG")) = 0.01;
  model.nweights_(model.nameMapping("nmap").toId("SeedNPG")) = 100;
  asp_.setModel(model);
  cout << model << endl;
  asp_.writeGraphviz("graphvis");
  asp_.setDebug(false);
  
  hrt_.start();
}

void DetectionVisualizer::callback(const sentinel::Detection& msg)
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

  cv::Mat1b indices_mask(cv::Size(msg.width, msg.height), 0);
  for(size_t i = 0; i < msg.indices.size(); ++i) {
    uint32_t idx = msg.indices[i];
    int y = idx / depth_vis_.cols;
    int x = depth_vis_.cols - 1 - (idx - y * depth_vis_.cols);
    indices_mask(y, x) = 255;
  }

  cv::Mat1f depth(color_vis_.size(), 0);
  for(size_t i = 0; i < msg.indices.size(); ++i) {
    uint32_t idx = msg.indices[i];
    int y = idx / depth.cols;
    int x = depth.cols - 1 - (idx - y * depth.cols);
    depth(y, x) = msg.depth[i] * 0.001;
  }
  
  // -- Output a framerate estimate.
  timestamps_.push_back(hrt_.getSeconds());
  if(timestamps_.size() > 100)
    timestamps_.pop_front();
  // if(timestamps_.size() == 100)
  //   cout << "FPS: " << timestamps_.size() / (timestamps_.back() - timestamps_.front()) << endl;

  // -- Get a boundary mask in block space.
  int blocks_per_row = msg.width / msg.width_step;
  int blocks_per_col = msg.height / msg.height_step;
  
  cv::Mat1b fg_block_img(cv::Size(blocks_per_row, blocks_per_col), 0);
  for(size_t i = 0; i < msg.fg_indices.size(); ++i) {
    uint32_t idx = msg.fg_indices[i];
    int y = idx / msg.width;
    int x = msg.width - 1 - (idx - y * msg.width);
    int r = (double)(y - msg.height_step / 2) / msg.height_step + 0.5;
    int c = (double)(x - msg.width_step / 2) / msg.width_step + 0.5;
    ROS_ASSERT(r >= 0 && r < blocks_per_col);
    ROS_ASSERT(c >= 0 && c < blocks_per_row);
    fg_block_img(r, c) = 255;
  }
  
  cv::Mat1b bg_block_img(fg_block_img.size(), 0);
  for(int y = 0; y < fg_block_img.rows; ++y)
    for(int x = 0; x < fg_block_img.cols; ++x)
      if(fg_block_img(y, x) == 0)
        bg_block_img(y, x) = 255;




  cv::Mat1b rough_fg_mask = fg_block_img.clone();
  cv::Mat1b rough_bg_mask = bg_block_img.clone();
  int iters = 1;
  cv::dilate(rough_fg_mask, rough_fg_mask, cv::Mat(), cv::Point(-1, -1), iters);
  cv::dilate(rough_bg_mask, rough_bg_mask, cv::Mat(), cv::Point(-1, -1), iters);
  
  cv::Mat1b block_mask(rough_bg_mask.size(), 0);
  for(int y = 0; y < block_mask.rows; ++y)
    for(int x = 0; x < block_mask.cols; ++x)
      if(rough_fg_mask(y, x) == 255 && rough_bg_mask(y, x) == 255)
        block_mask(y, x) = 255;

  // Make the full-sized mask.
  // Only allow points with data in the mask.
  cv::Mat1b mask(cv::Size(msg.width, msg.height), 0);
  for(int y = 0; y < mask.rows; ++y)
    for(int x = 0; x < mask.cols; ++x)
      mask(y, x) = block_mask(y / msg.height_step, x / msg.width_step);
  for(int y = 0; y < mask.rows; ++y)
    for(int x = 0; x < mask.cols; ++x)
      if(indices_mask(y, x) == 0)
        mask(y, x) = 0;
  
  cv::imshow("rough_fg_mask", rough_fg_mask);
  cv::imshow("rough_bg_mask", rough_bg_mask);
  cv::imshow("mask", mask);
  
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

  // -- Construct seed image.
  cv::Mat1b seed(color_vis_.size(), 127);
  for(size_t i = 0; i < msg.fg_indices.size(); ++i) {
    uint32_t idx = msg.fg_indices[i];
    int y = idx / msg.width;
    int x = msg.width - 1 - (idx - y * msg.width);
    seed(y, x) = 255;
  }
  for(size_t i = 0; i < msg.bg_fringe_indices.size(); ++i) {
    uint32_t idx = msg.bg_fringe_indices[i];
    int y = idx / msg.width;
    int x = msg.width - 1 - (idx - y * msg.width);
    seed(y, x) = 0;
  }

  // -- Display.
  cv::Mat3b depth_vis_alt = depth_vis_.clone();
  for(int y = 0; y < fg_block_img.rows; ++y) {
    for(int x = 0; x < fg_block_img.cols; ++x) {
      if(fg_block_img(y, x) == 255)
        cv::circle(depth_vis_alt,
                   cv::Point(((float)x + 0.5) * msg.width_step, ((float)y + 0.5) * msg.height_step),
                   1, cv::Scalar(0, 255, 0), -1);
      else
        cv::circle(depth_vis_alt,
                   cv::Point(((float)x + 0.5) * msg.width_step, ((float)y + 0.5) * msg.height_step),
                   1, cv::Scalar(255, 0, 0), -1);
    }
  }
  cv::Mat3b depth_vis_alt_scaled;
  cv::resize(depth_vis_alt, depth_vis_alt_scaled, depth_vis_alt.size() * g_scaling, cv::INTER_NEAREST);
  cv::imshow("depth_vis_alt", depth_vis_alt_scaled);
  
  
  cv::Mat3b depth_scaled;
  cv::resize(depth_vis_, depth_scaled, depth_vis_.size() * g_scaling, cv::INTER_NEAREST);
  for(size_t i = 0; i < msg.fg_indices.size(); ++i) {
    uint32_t idx = msg.fg_indices[i];
    int y = idx / msg.width;
    int x = msg.width - 1 - (idx - y * msg.width);
    cv::circle(depth_scaled, cv::Point(x, y) * g_scaling, 1, cv::Scalar(0, 255, 0), -1);
  }
  for(size_t i = 0; i < msg.bg_fringe_indices.size(); ++i) {
    uint32_t idx = msg.bg_fringe_indices[i];
    int y = idx / msg.width;
    int x = msg.width - 1 - (idx - y * msg.width);
    cv::circle(depth_scaled, cv::Point(x, y) * g_scaling, 1, cv::Scalar(255, 0, 0), -1);
  }

  cv::imshow("depth", depth_scaled);
  cv::Mat3b color_scaled;
  cv::resize(color_vis_, color_scaled, cv::Size(640, 480), cv::INTER_NEAREST);
  cv::imshow("color", color_scaled);

  // -- Compute a more detailed foreground / background mask.
  //    Will be 255 for foreground points.
  // asp_.setInput("ImageEntryPoint", color_vis_);
  // asp_.setInput("MaskEntryPoint", mask);
  // asp_.setInput("SeedEntryPoint", seed);
  
  // asp_.setInput("DepthEntryPoint", depth);
  // cv::Mat1b foreground(cv::Size(msg.width, msg.height), 0);
  // hrt.reset("asp"); hrt.start();
  // asp_.segment(&foreground);
  // hrt.stop(); cout << hrt.reportMilliseconds() << endl;
  // cout << asp_.reportTiming() << endl;

  // for(int y = 0; y < foreground.rows; ++y)
  //   for(int x = 0; x < foreground.cols; ++x)
  //     if(indices_mask(y, x) != 255 || depth(y, x) == 0)
  //       foreground(y, x) = 0;

  // for(int y = 0; y < foreground.rows; ++y) {
  //   for(int x = 0; x < foreground.cols; ++x) {
  //     if(mask(y, x) == 0 && fg_block_img(y / msg.height_step, x / msg.width_step) == 255)
  //       foreground(y, x) = 255;
  //   }
  // }

  // cv::imshow("ASP Foreground", foreground);


  // -- Try simple floodfilling to get the foreground.
  cv::Mat1i simple_foreground_assignments(cv::Size(msg.width, msg.height), -3);
  for(int y = 0; y < simple_foreground_assignments.rows; ++y)
    for(int x = 0; x < simple_foreground_assignments.cols; ++x)
      if(depth(y, x) == 0)
        simple_foreground_assignments(y, x) = -1;
  for(size_t i = 0; i < msg.fg_indices.size(); ++i) {
    uint32_t idx = msg.fg_indices[i];
    int y = idx / msg.width;
    int x = msg.width - 1 - (idx - y * msg.width);
    flood(depth, 0.2, cv::Point(x, y), 0, &simple_foreground_assignments);
  }
  cv::Mat1b simple_foreground(cv::Size(msg.width, msg.height), 0);
  for(int y = 0; y < simple_foreground_assignments.rows; ++y)
    for(int x = 0; x < simple_foreground_assignments.cols; ++x)
      if(simple_foreground_assignments(y, x) == 0)
        simple_foreground(y, x) = 255;

  cv::imshow("Simple Foreground", simple_foreground);

  // -- Try bilateral filtering to get the foreground.
  hrt.reset("Bilateral"); hrt.start();
  double sigma_p = 10;
  double sigma_d = 0.05;
  cv::Mat1b bilateral_foreground(cv::Size(msg.width, msg.height), 0);
  vector<cv::Point2i> keypoints;
  for(int y = msg.height_step / 2; y < bilateral_foreground.rows - msg.height_step / 2; ++y) {
    for(int x = msg.width_step / 2; x < bilateral_foreground.cols - msg.width_step / 2; ++x) {
      if(depth(y, x) == 0)
        continue;
      
      int r = (double)(y - msg.height_step / 2) / msg.height_step + 0.5;
      int c = (double)(x - msg.width_step / 2) / msg.width_step + 0.5;

      // for all neighboring fg / bg markers
      // compute image distance and 3d distance, sum up bilateral products
      double val = 0;
      for(int r2 = r-1; r2 <= r+1; ++r2) {
        for(int c2 = c-1; c2 <= c+1; ++c2) {
          if(r2 < 0 || r2 >= fg_block_img.rows ||
             c2 < 0 || c2 >= fg_block_img.cols)
          {
            continue;
          }

          cv::Point2i pt;
          pt.y = r2 * msg.height_step + msg.height_step / 2;
          pt.x = c2 * msg.width_step + msg.width_step / 2;
          //cout << pt << endl;
          
          double mult = -1;
          if(fg_block_img(r2, c2) == 255)
            mult = 1;
          double img_coef = exp(-sqrt((x - pt.x)*(x - pt.x) + (y - pt.y)*(y - pt.y)) / sigma_p);
          double z = depth(pt);
          double depth_coef = exp(-fabs(depth(y, x) - z) / sigma_d);
          double term = mult * img_coef * depth_coef;
          val += term;

          if(y == g_y && x == g_x) {
            cout << "Point " << x << " " << y << " with depth " << depth(y, x) << "; keypoint " << pt
                 << " with depth " << depth(pt) << " and label " << (int)fg_block_img(r2, c2)
                 <<"; img_coef: " << img_coef << ", depth_coef: " << depth_coef
                 << ", term : " << term << endl;
          }
        }
      }

      if(val > 0)
        bilateral_foreground(y, x) = 255;
    }
  }
  //hrt.stop(); cout << hrt.reportMilliseconds() << endl;
  cv::Mat1b bilateral_foreground_scaled;
  cv::resize(bilateral_foreground, bilateral_foreground_scaled,
             bilateral_foreground.size() * g_scaling, cv::INTER_NEAREST);
  cv::imshow("bilateral_foreground", bilateral_foreground_scaled);
  
  // -- Run connected components on the depth data in the foreground.
  for(int y = 0; y < simple_foreground.rows; ++y)
    for(int x = 0; x < simple_foreground.cols; ++x)
      if(simple_foreground(y, x) == 0)
        depth(y, x) = 0;

  ROS_ASSERT(msg.width % msg.width_step == 0 && msg.height % msg.height_step == 0);
  cv::Mat1i blobs(cv::Size(msg.width, msg.height), 0);
  hrt.reset("clustering"); hrt.start();
  cluster(depth, 0.2, &blobs);
  //hrt.stop(); cout << hrt.reportMilliseconds() << endl;

  cv::imshow("Clustering", colorAssignments(blobs));
  cv::waitKey(5);
}

