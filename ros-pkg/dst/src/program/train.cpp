#include <dst/struct_svm.h>
#include <eigen_extensions/eigen_extensions.h>

#define MARGIN_RESCALING (getenv("MARGIN_RESCALING"))

using namespace std;
using namespace dst;

string usageString()
{
  ostringstream oss;
  oss << "Usage: ./train TRAINING_DIR [TRAINING_DIR ...] OUTPUT_WEIGHTS" << endl;
  return oss.str();
}

int main(int argc, char** argv)
{
  if(argc < 3) {
    cout << usageString() << endl;
    return 0;
  }

  string savepath = argv[argc-1];
  ROS_ASSERT(savepath.size() > 7);
  ROS_ASSERT(savepath.substr(savepath.size() - 8).compare(".eig.txt") == 0);

  if(getenv("ROBUST"))
    cout << "Using robust training." << endl;
  else
    cout << "Using frame-to-frame training." << endl;    

  double c = 1.0;
  if(getenv("C_VALUE"))
    c = atof(getenv("C_VALUE"));
  cout << "Using c = " << c << endl;

  if(MARGIN_RESCALING)
    cout << "Using margin rescaling." << endl;
  else
    cout << "Not using margin rescaling." << endl;
  
  StructSVM learner(c, 1, MARGIN_RESCALING);
  for(int i = 1; i < argc - 1; ++i) 
    learner.loadSequences(argv[i]);

  Eigen::VectorXd weights;
  if(getenv("ROBUST"))
    weights = learner.trainRobust2();
  else
    weights = learner.train();
  
  cout << "Learned weights " << endl << weights.transpose() << endl;
  eigen_extensions::saveASCII(weights, savepath);
  cout << "Saved to " << savepath << endl;
  
  return 0;
}
