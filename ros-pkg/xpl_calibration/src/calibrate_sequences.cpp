#include <eigen_extensions/eigen_extensions.h>
#include <xpl_calibration/xpl_calibrator.h>
#include <xpl_calibration/xpl_calibrator_orb.h>

using namespace std;
using namespace pcl;
using namespace rgbd;

#define USE_PLANES (getenv("USE_PLANES") ? atoi(getenv("USE_PLANES")) : 0)

string usageString()
{
  ostringstream oss;
  oss << "Usage: calibrate_sequences SEQ SEQ" << endl;
  return oss.str();
}

int main(int argc, char** argv)
{
  if(argc != 3) {
    cout << usageString() << endl;
    return 0;
  }

  Sequence::Ptr reference(new Sequence);
  Sequence::Ptr target(new Sequence);
  reference->load(argv[1]);
  target->load(argv[2]);

  Eigen::Affine3f transform;
  if(USE_PLANES) { 
    XplCalibrator xc;
    transform = xc.calibrate(reference, target);
  }
  else {
    XplCalibratorOrb xc;
    transform = xc.calibrate(reference, target);
  }

  cout << transform.matrix() << endl;
  eigen_extensions::saveASCII(transform.matrix(), "best_transform.eig.txt");
  
  return 0;
}
