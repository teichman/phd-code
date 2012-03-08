#include <Eigen/Geometry>
#include <iostream>
#include <eigen_extensions/eigen_extensions.h>

using namespace std;
using namespace Eigen;

string usageString()
{
  ostringstream oss;
  oss << "Usage: evaluate GROUND_TRUTH_EIG ESTIMATED_EIG" << endl;
  return oss.str();
}

int main(int argc, char** argv)
{
  if(argc != 3) {
    cout << usageString() << endl;
    return 0;
  }

  MatrixXd gt;
  MatrixXd est;
  eigen_extensions::loadASCII(argv[1], &gt);
  eigen_extensions::loadASCII(argv[2], &est);
  cout << "Ground truth: " << endl << gt << endl;
  cout << endl;
  cout << "Estimated: " << endl << est << endl;

  cout << "Frobenius: " << (gt - est).norm() << endl;
  cout << "Translation: " << (gt.block<3, 1>(0, 3) - est.block<3, 1>(0, 3)).norm() << endl;

  return 0;
}
