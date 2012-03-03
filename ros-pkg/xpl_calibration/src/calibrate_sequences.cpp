#include <eigen_extensions/eigen_extensions.h>
#include <xpl_calibration/calibration_pipeline_orb.h>

using namespace std;
using namespace pcl;
using namespace rgbd;

#define NUM_THREADS (getenv("NUM_THREADS") ? atoi(getenv("NUM_THREADS")) : 1)

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

  Sequence::Ptr seq0(new Sequence);
  Sequence::Ptr seq1(new Sequence);
  seq0->load(argv[1]);
  seq1->load(argv[2]);

  Eigen::Affine3f transform;
  CalibrationPipelineOrb cp(NUM_THREADS);
  transform = cp.calibrate(seq0, seq1);
  
  cout << transform.matrix() << endl;
  eigen_extensions::saveASCII(transform.matrix(), "best_transform.eig.txt");
  
  return 0;
}
