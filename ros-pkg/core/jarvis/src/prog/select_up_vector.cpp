#include <boost/program_options.hpp>
#include <eigen_extensions/eigen_extensions.h>
#include <openni2_interface/openni2_interface.h>
#include <openni2_interface/openni_helpers.h>
#include <opencv2/highgui/highgui.hpp>
#include <image_labeler/opencv_view.h>
#include <stream_sequence/frame_projector.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/ransac.h>


using namespace std;
using namespace Eigen;
using namespace clams;

class UpSelector : public OpenNI2Handler, public OpenCVViewDelegate, public Agent
{
public:
  UpSelector(std::string output_path);
  ~UpSelector();
  void rgbdCallback(openni::VideoFrameRef color, openni::VideoFrameRef depth,
                    size_t frame_id, double timestamp);
  void _run();

protected:
  OpenNI2Interface oni_;
  OpenCVView view_;
  std::string output_path_;
  int radius_;
  cv::Mat1b selection_;
  openni::VideoFrameRef depth_;

  void mouseEvent(int event, int x, int y, int flags, void* param);
  void handleKeypress(char key);
  void selectPlane();
  void clear();
};

UpSelector::~UpSelector()
{
}

UpSelector::UpSelector(std::string output_path) :
  oni_(OpenNI2Interface::VGA, OpenNI2Interface::VGA),
  view_("PlaneSelector"),
  output_path_(output_path),
  radius_(10)
{
  view_.setDelegate(this);
  oni_.setHandler(this);
}

void UpSelector::mouseEvent(int event, int x, int y, int flags, void* param)
{
  // -- Left click to add to plane selection.
  if(flags & CV_EVENT_FLAG_LBUTTON) {
    for(int i = x - radius_; i <= x + radius_; ++i) { 
      for(int j = y - radius_; j <= y + radius_; ++j) {
        if(i >= 0 && i < selection_.cols &&
           j >= 0 && j < selection_.rows) {
          selection_(j, i) = 255;
        }
      }
    }
  }
}

void UpSelector::rgbdCallback(openni::VideoFrameRef color, openni::VideoFrameRef depth,
                              size_t frame_id, double timestamp)
{
  //cv::Mat3b img = colorize(oniDepthToEigen(depth), 0.5, 7);
  cv::Mat3b img = oniToCV(color);
  
  if(selection_.rows != img.rows)
    selection_ = cv::Mat1b(img.size(), 0);

  cv::Mat3b vis = img.clone();
  for(int y = 0; y < vis.rows; ++y)
    for(int x = 0; x < vis.cols; ++x)
      if(selection_(y, x) > 0)
        vis(y, x) = cv::Vec3b(0, 255, 0);
  
  view_.updateImage(vis);
  char key = view_.cvWaitKey(5);
  handleKeypress(key);

  scopeLockWrite;
  depth_ = depth;
}

void UpSelector::handleKeypress(char key)
{
  switch(key) {
  case 'q':
    oni_.stop();
    break;
  case ' ':
    selectPlane();
    break;
  case 'c':
    clear();
    break;
  default:
    break;
  }
}

void UpSelector::_run()
{
  oni_.run();
}

void UpSelector::selectPlane()
{
  scopeLockRead;
  
  // -- Make a pointcloud with just the selected points.
  FrameProjector proj;
  proj.width_ = selection_.cols;
  proj.height_ = selection_.rows;
  proj.cx_ = (double)selection_.cols / 2;
  proj.cy_ = (double)selection_.rows / 2;
  ROS_ASSERT(selection_.cols = 640);
  proj.fx_ = 525;
  proj.fy_ = 525;

  // This is really nasty.  I should not be mixing clams code and this stuff.
  Frame frame;
  frame.img_ = cv::Mat3b(selection_.size(), cv::Vec3b(0, 0, 0));
  ROS_ASSERT(depth_.getVideoMode().getPixelFormat() == openni::PIXEL_FORMAT_DEPTH_1_MM);
  frame.depth_ = clams::DepthMatPtr(new clams::DepthMat(depth_.getHeight(), depth_.getWidth()));
  ushort* data = (ushort*)depth_.getData();
  int idx = 0;
  for(int y = 0; y < frame.depth_->rows(); ++y)
    for(int x = 0; x < frame.depth_->cols(); ++x, ++idx)
      frame.depth_->coeffRef(y,x) = data[idx];

  Cloud::Ptr pcd(new Cloud);
  pcd->reserve(selection_.rows * selection_.cols);
  for(int y = 0; y < selection_.rows; ++y) {
    for(int x = 0; x < selection_.cols; ++x) {
      if(selection_(y, x) > 0) {
        ProjectivePoint ppt;
        ppt.u_ = x;
        ppt.v_ = y;
        ppt.z_ = frame.depth_->coeffRef(y, x);
        Point pt;
        proj.project(ppt, &pt);
        pcd->push_back(pt);
      }
    }
  }
          
  // -- Fit the best plane.
  double tol = 0.03;
  pcl::SampleConsensusModelPlane<Point>::Ptr plane(new pcl::SampleConsensusModelPlane<Point>(pcd));
  pcl::RandomSampleConsensus<Point> ransac(plane);
  ransac.setDistanceThreshold(tol);
  ransac.computeModel();
  std::vector<int> inliers;
  ransac.getInliers(inliers);  // inliers indexes into pcd
  VectorXf raw_coefs;
  ransac.getModelCoefficients(raw_coefs);
  VectorXf coefs;
  plane->optimizeModelCoefficients(inliers, raw_coefs, coefs);

  cout << "Plane fit coefficients: " << coefs.transpose() << endl;
  cout << "Num inliers among pcd points: " << inliers.size() << endl;
  // for(size_t i = 0; i < inliers.size(); ++i)
  //   cout << "Pt: " << (*pcd)[inliers[i]].getVector4fMap().transpose() << ", a^T pt: " << coefs.dot((*pcd)[inliers[i]].getVector4fMap()) << endl;

  // -- Update the selection.
  selection_ = 0;
  proj.frameToCloud(frame, pcd.get());
  
  for(size_t i = 0; i < pcd->size(); ++i) {
    const Point& pt = pcd->at(i);
    if(fabs(coefs.dot(pt.getVector4fMap())) < tol) {
      ProjectivePoint ppt;
      proj.project(pt, &ppt);
      selection_(ppt.v_, ppt.u_) = 255;
    }
  }

  // -- Save.
  VectorXf save = coefs.head(3);
  eigen_extensions::saveASCII(save, output_path_);
}

void UpSelector::clear()
{
  selection_ = 0;
}

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  string output_path;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("output-path", bpo::value(&output_path)->default_value("up.eig.txt"), "")
    ;

  p.add("output-path", 1);
  
  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << argv[0] << " [OPTS] OUTPUT_PATH" << endl; 
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  cout << "Saving to " << output_path << endl;

  UpSelector ups(output_path);
  ups.run();
  
  return 0;
}
