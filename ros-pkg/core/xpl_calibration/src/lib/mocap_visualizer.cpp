#include <xpl_calibration/mocap_visualizer.h>
#include <pcl/common/transformation_from_correspondences.h>

using namespace std;
using namespace Eigen;
using namespace rgbd;

MocapVisualizer::MocapVisualizer(const TRCParser& trc,
                                 rgbd::StreamSequence::ConstPtr sseq,
                                 double tol) :
  trc_(trc),
  sseq_(sseq),
  tol_(tol),
  trc_idx_(0),
  xpl_idx_(0),
  sync_(0),
  sseq_start_(sseq_->getCloud(0)->header.stamp.toSec())
{
}

void MocapVisualizer::getTRCPoints(const rgbd::Cloud& frame,
                                   rgbd::Cloud* camera,
                                   rgbd::Cloud* checker) const
{
  // -- Find the camera.
  for(size_t i = 0; i < frame.size(); ++i) {
    int cam2 = -1;
    int cam3 = -1;
    for(size_t j = 0; j < frame.size(); ++j) {
      if(fabs(pcl::euclideanDistance(frame[i], frame[j]) - 0.115) < tol_)
        cam2 = j;
      if(fabs(pcl::euclideanDistance(frame[i], frame[j]) - 0.1325) < tol_)
        cam3 = j;
    }
    if(cam2 == -1 && cam3 == -1)
      checker->push_back(frame[i]);
    if(cam2 != -1 && cam3 != -1) {
      camera->push_back(frame[i]);
      camera->push_back(frame[cam2]);
      camera->push_back(frame[cam3]);
    }
  }

  // -- Color the camera points.
  for(size_t i = 0; i < camera->size(); ++i) {
    camera->at(i).r = 0;
    camera->at(i).g = 255;
    camera->at(i).b = 0;
  }
  for(size_t i = 0; i < checker->size(); ++i) {
    checker->at(i).r = 0;
    checker->at(i).g = 0;
    checker->at(i).b = 255;
  }
}

void MocapVisualizer::getXPLPoints(rgbd::Cloud* xpl) const
{
  xpl->clear();

  MocapDetector md;
  Point tl, tr, bl, br;
  bool found = md.locatePoints(sseq_, xpl_idx_, tl, tr, bl, br);
  if(!found)
    return;

  xpl->push_back(tl);
  xpl->push_back(tr);
  xpl->push_back(bl);
  xpl->push_back(br);
  for(size_t i = 0; i < xpl->size(); ++i) {
    xpl->at(i).r = 255;
    xpl->at(i).g = 0;
    xpl->at(i).b = 0;
  }
}

void MocapVisualizer::run()
{
  Cloud::Ptr vis(new Cloud);
  vw_.vis_.addText("aoeu", 10, 10, 0, 0, 0, "frame_number");
    
  while(true) {
    cout << "------------------------------" << endl;
    cout << "xpl #" << xpl_idx_ << ", trc #" << trc_idx_ << endl;
    cout << "xpl ts: " << sseq_->timestamps_[xpl_idx_] - sseq_start_ << endl;
    cv::imshow("img", sseq_->getImage(xpl_idx_));
    cv::waitKey(2);
    
    const Cloud& frame = *trc_.frames_[trc_idx_];
    rgbd::Cloud camera;
    rgbd::Cloud checker;
    getTRCPoints(frame, &camera, &checker);
    rgbd::Cloud xpl;
    getXPLPoints(&xpl);
    cout << "#xpl: " << xpl.size() << ", #trc_cam: " << camera.size() << ", #trc_checker: " << checker.size() << endl;

    
    // -- Visualize.
    Affine3f transform = Affine3f::Identity();
    if(camera.size() == 3) { 
      transform = getWorldToCameraTransform(camera);
      *vis = camera;
      *vis += checker;
      pcl::transformPointCloud(*vis, *vis, transform);
      *vis += xpl;

      vw_.showCloud(vis);
      ostringstream oss;
      oss << "XPL frame " << setw(4) << setfill('0') << xpl_idx_;
      vw_.vis_.removeShape("frame_number");
      vw_.vis_.addText(oss.str(), 10, 10, 0, 0, 0, "frame_number");
    }
    else {
      vw_.vis_.removeShape("frame_number");
      vw_.vis_.addText("Failed to find camera in mocap system", 10, 10, 0, 0, 0, "frame_number");
      vis->clear();
    }
    
    char key = vw_.waitKey();
    switch(key) {
    case 27:
      return;
      break;
    case ',':
      incrementXPL(-1);
      break;
    case '.':
      incrementXPL(1);
      break;
    case '<':
      incrementXPL(-100);
      break;
    case '>':
      incrementXPL(100);
      break;
    case '[':
      incrementSync(-1.0 / 120.0);
      break;
    case ']':
      incrementSync(1.0 / 120.0);
      break;
    case '{':
      incrementSync(-10.0 / 120.0);
      break;
    case '}':
      incrementSync(10.0 / 120.0);
      break;
    }
  }
}

void MocapVisualizer::incrementSync(double val)
{
  sync_ += val;
  cout << "New sync offset: " << sync_ << endl;
  incrementXPL(0);
}

void MocapVisualizer::incrementXPL(int val)
{
  xpl_idx_ += val;
  if(xpl_idx_ < 0)
    xpl_idx_ = 0;
  if(xpl_idx_ >= (int)sseq_->size())
    xpl_idx_ = sseq_->size() - 1;

  // -- Seek for the closest trc_idx_.
  double min = numeric_limits<double>::max();
  double ts = sseq_->timestamps_[xpl_idx_] - sseq_start_ + sync_;
  int best = -1;
  for(size_t i = 0; i < trc_.frames_.size(); ++i) {
    double dt = fabs(ts - trc_.frames_[i]->header.stamp.toSec());
    if(dt < min) {
      min = dt;
      best = i;
    }
  }

  trc_idx_ = best;
}

Eigen::Affine3f MocapVisualizer::getWorldToCameraTransform(const rgbd::Cloud& camera) const
{
  ROS_ASSERT(camera.size() == 3);

  Vector3f c0 = camera[0].getVector3fMap();
  Vector3f c1 = camera[1].getVector3fMap();
  Vector3f c2 = camera[2].getVector3fMap();
    
  Vector3f x = c0 - c1;
  Vector3f z = c1 - c2;
  Vector3f y = -x.cross(z);
  x.normalize();
  y.normalize();
  z.normalize();

  pcl::TransformationFromCorrespondences tfc;
  tfc.add(c0 + x, Vector3f::UnitX());
  tfc.add(c0 + y, Vector3f::UnitY());
  tfc.add(c0 + z, Vector3f::UnitZ());
   
  Eigen::Affine3f transform = tfc.getTransformation();

  // Measured offset from camera marker 0 to camera center of projection.
  // Rough.
  transform(0, 3) += 0.04;
  transform(1, 3) += -0.02;
  transform(2, 3) += 0.03;

  return transform;
}
