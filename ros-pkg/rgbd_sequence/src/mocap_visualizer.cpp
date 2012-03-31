#include <rgbd_sequence/mocap_visualizer.h>
#include <pcl/common/transformation_from_correspondences.h>

using namespace std;
using namespace Eigen;

namespace rgbd
{

  MocapVisualizer::MocapVisualizer(const TRCParser& trc,
				   const std::vector<rgbd::Cloud::Ptr>& xpl,
				   double tol) :
    trc_(trc),
    xpl_(xpl),
    tol_(tol),
    trc_idx_(0),
    xpl_idx_(0),
    sync_(0)
  {
    // -- Make the XPL data start at zero.
    double offset = xpl_[0]->header.stamp.toSec();
    for(size_t i = 0; i < xpl_.size(); ++i) {
      double ts = xpl_[i]->header.stamp.toSec() - offset;
      xpl_[i]->header.stamp.fromSec(ts);
    }

    // -- Make the XPL data red.
    for(size_t i = 0; i < xpl_.size(); ++i) {
      for(size_t j = 0; j < xpl_[i]->size(); ++j) {
	xpl_[i]->at(j).r = 255;
	xpl_[i]->at(j).g = 0;
	xpl_[i]->at(j).b = 0;
      }
    }
  }

  void MocapVisualizer::getPointTypes(const rgbd::Cloud& frame,
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
  
  void MocapVisualizer::run()
  {
    Cloud::Ptr vis(new Cloud);
    
    while(true) {
      cout << "trc_idx_: " << trc_idx_ << ", xpl_idx_: " << xpl_idx_ << endl;
      const Cloud& frame = *trc_.frames_[trc_idx_];
      rgbd::Cloud camera;
      rgbd::Cloud checker;
      getPointTypes(frame, &camera, &checker);
      cout << "Got " << camera.size() << " camera points, " << checker.size() << " checker points." << endl;
      
      Affine3f transform = Affine3f::Identity();
      if(camera.size() == 3)
	transform = getWorldToCameraTransform(camera);
      else
	cout << "Failed to find camera." << endl;
            
      // -- Visualize.
      cout << "Showing frame " << trc_idx_ << endl;
      *vis = camera;
      *vis += checker;
      ROS_WARN_ONCE("Applying transform to fabricated XPL data.");
      *vis += *xpl_[xpl_idx_]; // This should come after the transform call.
      pcl::transformPointCloud(*vis, *vis, transform);

      vw_.showCloud(vis);
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
	incrementSync(100.0 / 120.0);
	break;
      case '}':
	incrementSync(100.0 / 120.0);
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
    if(xpl_idx_ >= (int)xpl_.size())
      xpl_idx_ = xpl_.size() - 1;

    // -- Seek for the closest trc_idx_.
    double min = numeric_limits<double>::max();
    double ts = xpl_[xpl_idx_]->header.stamp.toSec() + sync_;
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
  
} // namespace rgbd
