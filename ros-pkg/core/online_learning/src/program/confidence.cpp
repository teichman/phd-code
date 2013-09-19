#include <online_learning/evaluator.h>
#include <online_learning/training_buffer.h>

using namespace std;
namespace bfs = boost::filesystem;
using namespace odontomachus;

string usageString()
{
  ostringstream oss;
  oss << "Usage: confidence SEED_DATASET TEST_DATASET [ TEST_DATASET ... ] OUTPUT_PATH" << endl;
  return oss.str();
}

int main(int argc, char** argv)
{
  if(argc < 4) {
    cout << usageString() << endl;
    return 1;
  }

  string output_path = argv[argc-1];
  cout << "Saving output to " << output_path << endl;
  if(!bfs::exists(output_path))
    bfs::create_directory(output_path);
  
  // -- Load datasets.
  Dataset::Ptr seed(new Dataset);
  cout << "Loading seed dataset " << argv[1] << endl;
  seed->load(argv[1]);

  // -- Train classifier.
  TrainingBuffer buffer;
  buffer.applyNameMappings(*seed);
  buffer.init(5e5);
  buffer.merge(*seed);

  ProjectionSlicer::Ptr classifier(new ProjectionSlicer);
  classifier->initialize(500, *seed);
  LogisticStochasticTrainer::Ptr trainer(new LogisticStochasticTrainer);
  classifier->setTrainer(trainer);
  classifier->train(*buffer.getDataset());

  // -- Run evaluation.
  Dataset test;
  for(int i = 2; i < argc-1; ++i) {
    Dataset tmp;
    cout << "Loading test dataset " << argv[i] << endl;
    tmp.load(argv[i]);
    test += tmp;
  }
  ROS_ASSERT(seed->nameMappingsAreEqual(test));

  Evaluator ev(classifier);
  ev.evaluateParallel(test);
  ev.saveResults(output_path);
  ev.track_stats_.saveAccuracyVsConfidence(output_path + "/acc_vs_confidence", "", 100);

  return 0;  
}

