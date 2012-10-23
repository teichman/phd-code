#include <dst/realtime_interface.h>

using namespace std;
using namespace Eigen;
using namespace dst;

string usageString()
{
  ostringstream oss;
  oss << "Usage: real_time_interface WEIGHTS" << endl;
  return oss.str();
}

int main(int argc, char** argv)
{
  if(argc != 2) {
    cout << usageString() << endl;
    return 0;
  }

  int retval = system("killall XnSensorServer");
  cout << "killall XnSensorServer returned: " << retval << endl;
  
  VectorXd weights;
  eigen_extensions::loadASCII(argv[1], &weights);
  RealTimeInterface rti;
  rti.sp_.setWeights(weights, true);

  rti.run();
  
  return 0;
}
