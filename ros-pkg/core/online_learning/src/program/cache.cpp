#include <Eigen/Eigen>
#include <timer/timer.h>

using namespace std;
using namespace Eigen;

int main(int argc, char** argv)
{
  int num = 1000;
  Eigen::MatrixXd mat(num, num);
  mat.setRandom();

  HighResTimer hrt("rc");
  hrt.start();
  double val = 0;
  for(int i = 0; i < mat.rows(); ++i)
    for(int j = 0; j < mat.cols(); ++j)
      val += mat(i, j);
  cout << val << endl;
  hrt.stop();
  cout << hrt.report() << endl;

  hrt.reset("cr");
  hrt.start();
  val = 0;
  for(int j = 0; j < mat.cols(); ++j)
    for(int i = 0; i < mat.rows(); ++i)
      val += mat(i, j);
  cout << val << endl;
  hrt.stop();
  cout << hrt.report() << endl;
}
