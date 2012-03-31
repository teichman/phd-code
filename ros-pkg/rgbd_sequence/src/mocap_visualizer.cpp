#include <rgbd_sequence/mocap_visualizer.h>
#include <pcl/common/transformation_from_correspondences.h>

using namespace std;
using namespace Eigen;

namespace rgbd
{

  MocapVisualizer::MocapVisualizer(const TRCParser& trc, double tol) :
    trc_(trc),
    tol_(tol)
  {
  }

  void MocapVisualizer::run()
  {
    for(size_t i = 0; i < trc_.frames_.size(); ++i) {
      const Cloud& frame = *trc_.frames_[i];
      Cloud::Ptr vis(new Cloud);
      
      // -- Find the camera.
      vector<Point> camera;
      vector<Point> checker;
      for(size_t j = 0; j < frame.size(); ++j) {
	int cam2 = -1;
	int cam3 = -1;
	for(size_t k = 0; k < frame.size(); ++k) {
	  if(fabs(pcl::euclideanDistance(frame[j], frame[k]) - 0.115) < tol_)
	    cam2 = k;
	  if(fabs(pcl::euclideanDistance(frame[j], frame[k]) - 0.1325) < tol_)
	    cam3 = k;
	}
	if(cam2 == -1 && cam3 == -1)
	  checker.push_back(frame[j]);
	if(cam2 != -1 && cam3 != -1) {
	  camera.push_back(frame[j]);
	  camera.push_back(frame[cam2]);
	  camera.push_back(frame[cam3]);
	}
      }

      // -- Color the camera points.
      for(size_t j = 0; j < camera.size(); ++j) {
	camera[j].r = 0;
	camera[j].g = 255;
	camera[j].b = 0;
	vis->push_back(camera[j]);
      }

      for(size_t j = 0; j < checker.size(); ++j) {
	checker[j].r = 0;
	checker[j].g = 0;
	checker[j].b = 255;
	vis->push_back(checker[j]);
      }
      
      Affine3f transform = getWorldToCameraTransform(camera);
      pcl::transformPointCloud(*vis, *vis, transform);
      
      // -- Visualize.
      cout << "Showing frame " << i << endl;
      vw_.showCloud(vis);
      vw_.waitKey();
    }
  }

  Eigen::Affine3f MocapVisualizer::getWorldToCameraTransform(const std::vector<rgbd::Point>& camera) const
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

    return tfc.getTransformation();
  }
  
} // namespace rgbd
