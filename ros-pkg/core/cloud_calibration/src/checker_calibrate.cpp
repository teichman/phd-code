#include <cloud_calibration/checker_calibrator.h>
#include <eigen_extensions/eigen_extensions.h>
#include <rgbd_sequence/stream_sequence_base.h>

#define UNDISTORT (getenv("UNDISTORT") ? atoi(getenv("UNDISTORT")) : 0)

using namespace std;
using namespace cloud_calibration;

string usageString()
{
  ostringstream oss;
  oss << "Usage: checker_calibrate SEQ SEQ EIG [DT_THRESH]" << endl;
  oss << "Where SEQ is the reference StreamSequence" << endl;
  oss << "Where EIG is the location of the output transformation .eig.txt file" << endl;
  oss << "which brings the second sequence into the first sequence's frame" << endl;
  oss << "Where DT_THRESH is how close frames must be in time to be considered" << endl;
  return oss.str();
}

int main(int argc, char** argv)
{
  if(argc < 4) {
    cout << usageString() << endl;
    return 0;
  }
  cout << "Initializing seq 0" << endl;
  string seq_ref_dir = argv[1];
  rgbd::StreamSequenceBase::Ptr seq_ref = rgbd::StreamSequenceBase::initializeFromDirectory (seq_ref_dir);
  string seq_target_dir = argv[2];
  cout << "Initializing seq 1" << endl;
  rgbd::StreamSequenceBase::Ptr seq_target = rgbd::StreamSequenceBase::initializeFromDirectory (seq_target_dir);
  cout << "Loaded sequences!" << endl;
  seq_ref->setUndistort (UNDISTORT);
  seq_target->setUndistort (UNDISTORT);
  string eig_out = argv[3];
  double dt_thresh;
  if(argc == 5)
    dt_thresh = atof(argv[4]);
  else
    dt_thresh = 0.02;
  // Load sequence
  int rows = 6, cols = 8;
  CheckerCalibrator cc( rows, cols );
  float std_trans, std_rot;
  Eigen::Affine3f transf = cc.calibrate( seq_ref, seq_target, std_trans, std_rot, dt_thresh );
  cout << transf.matrix() << endl; 
  cout << "Saving to " << eig_out << endl;
  eigen_extensions::saveASCII( transf.matrix(), eig_out );
  cout << "Std_trans: " << std_trans << ", std_rot: " << std_rot << endl;
  Eigen::Vector2f stdev;
  stdev(0) = std_trans;
  stdev(1) = std_rot;
  string stdev_out = argv[3];
  stdev_out += "_std.eig.txt";
  eigen_extensions::saveASCII (stdev, stdev_out);


  return 0;
}
