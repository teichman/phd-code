#include <sentinel/sentinel.h>

using namespace std;

string usageString()
{
  ostringstream oss;
  oss << "Usage: record_stream NAME" << endl;
  oss << "  NAME is a string that describes this location." << endl;
  return oss.str();
}


int main(int argc, char** argv)
{
  if(argc != 2) {
    cout << usageString() << endl;
    return -1;
  }

  string name = argv[1];
  double update_interval = 1;
  double save_interval = 1;
  double threshold = 0.00001;
  int max_training_imgs = 180;
  Sentinel sen(name, update_interval, save_interval, max_training_imgs, threshold);
  sen.run();

  return 0;
}
