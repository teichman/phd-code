#include <cloud_calibration/checker_calibrator.h>
#include <eigen_extensions/eigen_extensions.h>

using namespace std;
using namespace cloud_calibration;

string usageString()
{
  ostringstream oss;
  oss << "Usage: checker_calibrate SEQ_REF SEQ_TARGET EIG [DT_THRESH]" << endl;
  oss << "Where SEQ_REF is the reference StreamSequence" << endl;
  oss << "Where SEQ_TARGET is the target StreamSequence" << endl;
  oss << "Where EIG is the location of the output transformation .eig.txt file" << endl;
  oss << "Where DT_THRESH is how close frames must be in time to be considered" << endl;
  return oss.str();
}

int main(int argc, char** argv)
{
  if(argc < 4) {
    cout << usageString() << endl;
    return 0;
  }
  string seq_ref_dir = argv[1];
  StreamSequence::Ptr seq_ref( new StreamSequence );
  seq_ref->load(seq_ref_dir);
  string seq_target_dir = argv[2];
  StreamSequence::Ptr seq_target( new StreamSequence );
  seq_target->load(seq_target_dir);
  string eig_out = argv[3];
  double dt_thresh;
  if(argc == 5)
    dt_thresh = atof(argv[4]);
  else
    dt_thresh = 0.02;
  // Load sequence
  int rows = 6, cols = 8;
  CheckerCalibrator cc( rows, cols );
  Eigen::Affine3f transf = cc.calibrate( seq_ref, seq_target, dt_thresh );
  cout << transf.matrix() << endl; 
  cout << "Saving to " << eig_out << endl;
  eigen_extensions::saveASCII( transf.matrix(), eig_out );

  return 0;
}
