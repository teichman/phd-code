#include <rgbd_sequence/primesense_model.h>

using namespace std;
using namespace rgbd;

int main(int argc, char** argv)
{
  PrimeSenseModel model;
  model.load(argv[1]);
  cout << model.status() << endl;
  return 0;
}
