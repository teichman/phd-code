#include <online_learning/tbssl.h>

using namespace std;

int main(int argc, char** argv)
{
  if(argc != 2) {
    cout << "Usage: learner_status LEARNER" << endl;
    return 1;
  }

  string path = argv[1];
  cout << "Checking status of OnlineLearner at " << path << endl;
  OnlineLearner learner((IfstreamWrapper(path)));
  cout << learner.status() << endl;
  
  return 0;
}
