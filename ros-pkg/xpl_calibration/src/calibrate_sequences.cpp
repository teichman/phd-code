#include <eigen_extensions/eigen_extensions.h>
#include <xpl_calibration/calibration_pipeline_orb.h>

using namespace std;
using namespace pcl;
using namespace rgbd;

#define NUM_THREADS (getenv("NUM_THREADS") ? atoi(getenv("NUM_THREADS")) : 1)

string usageString()
{
  ostringstream oss;
  oss << "Usage: calibrate_sequences SEQ SEQ [PIPELINE]" << endl;
  return oss.str();
}

int main(int argc, char** argv)
{
  if(argc != 3 && argc != 4) {
    cout << usageString() << endl;
    return 0;
  }

  StreamSequence::Ptr seq0(new StreamSequence);
  StreamSequence::Ptr seq1(new StreamSequence);
  cout << "Loading " << argv[1] << "." << endl;
  seq0->load(argv[1]);
  cout << "Loading " << argv[2] << "." << endl;
  seq1->load(argv[2]);

  string pipeline_path = "";
  if(argc == 4) { 
    pipeline_path = argv[3];
    cout << "Using custom Pipeline specification: " << pipeline_path << endl;
  }
  
  Eigen::Affine3f transform;
  CalibrationPipelineOrb cp(NUM_THREADS, pipeline_path);
  transform = cp.calibrate(seq0, seq1);
  
  cout << transform.matrix() << endl;
  eigen_extensions::saveASCII(transform.matrix(), "best_transform.eig.txt");
  
  return 0;
}
