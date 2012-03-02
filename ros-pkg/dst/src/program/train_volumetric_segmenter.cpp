#include <eigen_extensions/eigen_extensions.h>
#include <graphcuts/structural_svm.h>
#include <dst/volume_segmenter.h>

#define NUM_THREADS (getenv("NUM_THREADS") ? atoi(getenv("NUM_THREADS")) : 8)
#define C_VALUE (getenv("C_VALUE") ? atof(getenv("C_VALUE")) : 10)
#define TOL (getenv("TOL") ? atof(getenv("TOL")) : 0.2)
#define DEBUG_LEVEL (getenv("DEBUG_LEVEL") ? atoi(getenv("DEBUG_LEVEL")) : 0)

using namespace std;
using namespace dst;
using namespace Eigen;
namespace gc = graphcuts;

string usageString()
{
  ostringstream oss;
  oss << "Usage: train_volumetric_segmenter TRAINING_DIR [TRAINING_DIR ...] OUTPUT_WEIGHTS" << endl;
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
  cout << "C_VALUE = " << C_VALUE << endl;
  cout << "TOL = " << TOL << endl;

  vector<KinectSequence::Ptr> sequences;
  for(int i = 1; i < argc - 1; ++i) {
    cout << "Loading sequence " << argv[i] << endl;
    KinectSequence::Ptr seq(new KinectSequence);
    seq->load(argv[i]);
    sequences.push_back(seq);
  }

  vector<gc::PotentialsCache::Ptr> caches;
  vector<gc::VecXiPtr> labels;
  VolumeSegmenter vs;
  vs.cacheSequences(sequences, &caches, &labels);

  Eigen::VectorXd weights;
  gc::StructuralSVM learner(C_VALUE, TOL, NUM_THREADS, DEBUG_LEVEL);
  weights = learner.train(caches, labels);
  
  cout << "Learned weights " << endl << weights.transpose() << endl;
  eigen_extensions::saveASCII(weights, savepath);
  cout << "Saved to " << savepath << endl;

  return 0;
}

  
