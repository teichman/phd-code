#include <image_labeler/opencv_view.h>
#include <openni2_interface/openni2_interface.h>
#include <openni2_interface/openni_helpers.h>
#include <opencv2/features2d/features2d.hpp>
#include <agent/agent.h>
#include <timer/timer.h>
#include <boost/program_options.hpp>
#include <stream_sequence/frame_projector.h>

using namespace std;


typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<pcl::PointXYZRGB> Cloud;

class FeatureSet
{
public:
  Cloud::Ptr keycloud_;
  cv::Mat1f descriptors_;
  std::vector<cv::KeyPoint> keypoints_;

  void draw(cv::Mat3b img) const;
};

void FeatureSet::draw(cv::Mat3b img) const
{
  for(size_t i = 0; i < keypoints_.size(); ++i)
    cv::circle(img, keypoints_[i].pt, 2, cv::Scalar(0, 0, 255), -1);
}

void computeFeatures(clams::Frame frame, FeatureSet* fs)
{
  // Set up projector.
  clams::FrameProjector proj;
  proj.width_ = frame.img_.cols;
  proj.height_ = frame.img_.rows;
  proj.cx_ = proj.width_ / 2;
  proj.cy_ = proj.height_ / 2;
  ROS_ASSERT(frame.img_.cols == 640);
  proj.fx_ = 525;
  proj.fy_ = 525;

  // Compute features.
  cv::ORB orb(500);  // ~500 keypoints per image
  cv::Mat1b gray;
  cv::cvtColor(frame.img_, gray, CV_BGR2GRAY);
  cv::Mat1b mask = cv::Mat1b::zeros(frame.img_.rows, frame.img_.cols);
  mask = 255;  // Use everything.
  cv::Mat cvfeat;
  orb(gray, mask, fs->keypoints_, cvfeat);

  // Put into our format.
  fs->descriptors_ = cv::Mat1f(fs->keypoints_.size(), cvfeat.cols);
  for(size_t i = 0; i < fs->keypoints_.size(); i++)
    for(int j  = 0; j < cvfeat.cols; j++)
      fs->descriptors_(i, j) = cvfeat.at<uint8_t>(i, j);
  
  // Make keypoint cloud.
  fs->keycloud_ = Cloud::Ptr(new Cloud());
  fs->keycloud_->points.resize(fs->keypoints_.size());
  fs->keycloud_->height = 1;
  fs->keycloud_->width = fs->keypoints_.size();
  for(size_t i = 0; i < fs->keypoints_.size(); i++) {
    const cv::KeyPoint &kpt = fs->keypoints_[i];
    clams::ProjectivePoint kppt;
    kppt.u_ = kpt.pt.x;
    kppt.v_ = kpt.pt.y;
    kppt.z_ = frame.depth_->coeffRef(kppt.v_, kppt.u_);
    proj.project(kppt, &(fs->keycloud_->points[i]));
  }
}

//! Rigid Object Detector
class RodVisualizer : public OpenNI2Handler, public OpenCVViewDelegate, public Agent
{
public:
  RodVisualizer();
  
protected:
  OpenNI2Interface oni_;
  OpenCVView view_;
  std::vector<cv::Point2i> selected_points_;
  //! The thing we want to detect.
  FeatureSet model_;
  
  // Protected by mutex_.
  openni::VideoFrameRef current_color_;
  openni::VideoFrameRef current_depth_;
  
  // color and depth contain sensor timestamps.
  // timestamp contains the wall time, in seconds.
  void rgbdCallback(openni::VideoFrameRef color,
                    openni::VideoFrameRef depth,
                    size_t frame_id, double timestamp);
  void _run();
  void mouseEvent(int event, int x, int y, int flags, void* param);
  void keypress(char c);
  void visualizeFeatures();
  void drawSelection(cv::Mat3b vis) const;
  void detect();
  void select();
  clams::Frame getFrame();
};

RodVisualizer::RodVisualizer() :
  oni_(OpenNI2Interface::VGA, OpenNI2Interface::VGA),
  view_("Rigid Object Detector")
{
  view_.setDelegate(this);
}

void RodVisualizer::rgbdCallback(openni::VideoFrameRef color,
                                 openni::VideoFrameRef depth,
                                 size_t frame_id, double timestamp)
{
  lockWrite();
  current_color_ = color;
  current_depth_ = depth;
  unlockWrite();
  
  cv::Mat3b vis = oniToCV(color);
  drawSelection(vis);
  view_.updateImage(vis);
  char c = view_.cvWaitKey(2);
  if(c != -1)
    keypress(c);
}

void RodVisualizer::drawSelection(cv::Mat3b vis) const
{
  for(size_t i = 0; i < selected_points_.size(); ++i)
    cv::circle(vis, selected_points_[i], 4, cv::Scalar(0, 255, 0), -1);

  // Fill in box if we have the upper left and lower right corners.
  if(selected_points_.size() == 2) {
    cv::rectangle(vis, selected_points_[0], selected_points_[1], cv::Scalar(0, 255, 0), 1);
  }
}

void RodVisualizer::mouseEvent(int event, int x, int y, int flags, void* param)
{
  if(event == CV_EVENT_LBUTTONUP) {
    if(selected_points_.size() == 2)
      selected_points_.clear();
    
    selected_points_.push_back(cv::Point2i(x, y));
  }
}

void RodVisualizer::_run()
{
  oni_.setHandler(this);
  oni_.run();
}

void RodVisualizer::keypress(char c)
{
  switch(c) {
  case 'q':
    oni_.stop();
    break;
  case 'f':
    visualizeFeatures();
    break;
  case 's':
    select();
    break;
  case 'd':
    detect();
    break;
  default:
    break;
  }
}

void RodVisualizer::select()
{
  if(selected_points_.size() != 2) {
    cout << "You must select the template first." << endl;
    return;
  }

  // Compute features on the entire image.
  clams::Frame frame = getFrame();
  FeatureSet fs;
  computeFeatures(frame, &fs);
  cv::Point2i ul = selected_points_[0];
  cv::Point2i lr = selected_points_[1];

  // Get the FeatureSet for the object model.
  model_.keypoints_.clear();
  model_.keypoints_.reserve(fs.keypoints_.size());
  model_.keycloud_ = Cloud::Ptr(new Cloud);
  model_.keycloud_->clear();
  model_.keycloud_->reserve(fs.keypoints_.size());
  vector<int> indices;
  indices.reserve(fs.keypoints_.size());
  for(size_t i = 0; i < fs.keypoints_.size(); ++i) {
    cv::Point2f kpt = fs.keypoints_[i].pt;
    if(kpt.x > ul.x && kpt.x < lr.x &&
       kpt.y > ul.y && kpt.y < lr.y)
    {
      model_.keypoints_.push_back(fs.keypoints_[i]);
      model_.keycloud_->push_back(fs.keycloud_->at(i));
      indices.push_back(i);
    }
  }
  model_.descriptors_ = cv::Mat1f(model_.keypoints_.size(), fs.descriptors_.cols);
  for(int i = 0; i < model_.descriptors_.rows; ++i)
    for(int j  = 0; j < model_.descriptors_.cols; j++)
      model_.descriptors_(i, j) = fs.descriptors_.at<uint8_t>(indices[i], j);

  // Display.
  cv::Mat3b vis = frame.img_.clone();
  model_.draw(vis);
  cv::imshow("Model", vis);
}

void RodVisualizer::detect()
{

}

clams::Frame RodVisualizer::getFrame()
{
  clams::Frame frame;
  lockRead();
  frame.img_ = oniToCV(current_color_);
  frame.depth_ = oniDepthToEigenPtr(current_depth_);
  unlockRead();
  return frame;
}

void RodVisualizer::visualizeFeatures()
{
  clams::Frame frame = getFrame();
  FeatureSet fs;
  HighResTimer hrt("Feature computation"); hrt.start();
  computeFeatures(frame, &fs);
  hrt.stop(); cout << hrt.reportMilliseconds() << endl;
  cv::Mat3b vis = frame.img_.clone();
  fs.draw(vis);
  cv::imshow("Features", vis);
}

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  opts_desc.add_options()
    ("help,h", "produce help message")
    ;

  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << argv[0] << " [OPTS]" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  RodVisualizer rodvis;
  rodvis.run();
  
  return 0;
}
