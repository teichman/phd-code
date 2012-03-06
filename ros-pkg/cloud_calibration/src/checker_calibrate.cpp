#include <cloud_calibration/checker_calibrator.h>
#include <eigen_extensions/eigen_extensions.h>

using namespace std;
using namespace cloud_calibration;

string usageString()
{
  ostringstream oss;
  oss << "Usage: checker_calibrate sequence_dir rows cols checker_size" << endl;
  return oss.str();
}

int main(int argc, char** argv)
{
  if(argc != 5) {
    cout << usageString() << endl;
    return 0;
  }
  string sequence_dir = argv[1] ;
  int rows = atoi(argv[2]);
  int cols = atoi(argv[3]);
  float square_size = atof(argv[4]);
  // Load sequence
  MultiSequence::Ptr seq( new MultiSequence );
  seq->load( sequence_dir );
  MultiSequence::ConstPtr seq_const( seq );
  CheckerCalibrator cc( rows, cols, square_size );
  Eigen::Affine3f transf = cc.calibrate( seq_const, 1, 0 );
  cout << transf.matrix() << endl; 
  string savepath = "trans.eig.txt";
  cout << "Saving to " << savepath << endl;
  eigen_extensions::saveASCII( transf.matrix(), savepath );

  return 0;
}
