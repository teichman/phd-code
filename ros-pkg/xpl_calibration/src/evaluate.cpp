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

  Vector3d Tgt = gt.block<3, 1>(0, 3);
  Vector3d Test = est.block<3, 1>(0, 3);
  cout << "Euclidean distance: " << (Tgt - Test).norm() << endl;
  
  Matrix3d Rgt = gt.block<3, 3>(0, 0);
  Matrix3d Rest = est.block<3, 3>(0, 0);
  AngleAxisd aa(Rest.inverse() * Rgt);
  cout << "Angle: " << aa.angle() << endl;
  cout << "Axis: " << aa.axis().transpose() << endl;


  
  AngleAxisd final(AngleAxisd(M_PI/2.0, Vector3d::UnitZ()) * AngleAxisd(M_PI/2.0, Vector3d::UnitY()));
  cout << "jesse: " << final.angle() << endl;

  return 0;
}
