#include <xpl_calibration/object_matching_calibrator.h>

using namespace std;
using namespace Eigen;
using namespace pcl;
using namespace rgbd;

void ObjectMatchingCalibrator::compute()
{
  // const Sequence& seq = *pull<Sequence::ConstPtr>("Sequence");
  // const Objects& objects0 = *pull<const Objects*>("Objects0");
  // const Objects& objects1 = *pull<const Objects*>("Objects1");

  rough_transform_ = Affine3f::Identity();
  push<const Affine3f*>("RoughTransform", &rough_transform_);
}

void ObjectMatchingCalibrator::debug() const
{

}

