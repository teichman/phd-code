#include <boost/program_options.hpp>
#include <online_learning/evaluator.h>
#include <online_learning/training_buffer.h>

using namespace std;
namespace bfs = boost::filesystem;
namespace bpo = boost::program_options;
using namespace odontomachus;
using namespace Eigen;

void evaluate(ProjectionSlicer::ConstPtr classifier, const Dataset& test, std::string path)
{
  ROS_ASSERT(!bfs::exists(path));
  bfs::create_directory(path);

  Evaluator ev(classifier);
  ev.evaluateParallel(test);
  ev.saveResults(path);
  ev.track_stats_.saveAccuracyVsConfidence(path + "/acc_vs_confidence-track", "", 100);
  ev.frame_stats_.saveAccuracyVsConfidence(path + "/acc_vs_confidence-frame", "", 100);
}

void makeDataset(const std::vector< std::vector<VectorXf> >& cars,
		 const std::vector< std::vector<VectorXf> >& noncars,
		 Dataset* dataset)
{
  int num_frames = 0;
  for(size_t i = 0; i < cars.size(); ++i)
    num_frames += cars[i].size();
  for(size_t i = 0; i < noncars.size(); ++i)
    num_frames += noncars[i].size();

  dataset->labels_.resize(num_frames);
  dataset->descriptors_.resize(dataset->nameMapping("dmap").size(), num_frames);
  dataset->track_end_flags_.resize(num_frames);

  int idx = 0;
  for(size_t i = 0; i < cars.size(); ++i) {
    for(size_t j = 0; j < cars[i].size(); ++j, ++idx) {
      dataset->descriptors_.col(idx) = cars[i][j];
      dataset->labels_(idx) = dataset->nameMapping("cmap").toId("car");
      if(j == cars[i].size() - 1)
	dataset->track_end_flags_(idx) = 1;
      else
	dataset->track_end_flags_(idx) = 0;
    }
  }
  for(size_t i = 0; i < noncars.size(); ++i) {
    for(size_t j = 0; j < noncars[i].size(); ++j, ++idx) {
      dataset->descriptors_.col(idx) = noncars[i][j];
      dataset->labels_(idx) = -1;
      if(j == noncars[i].size() - 1)
	dataset->track_end_flags_(idx) = 1;
      else
	dataset->track_end_flags_(idx) = 0;
    }
  }
}

void splitSeed(const Dataset& seed, Dataset* train, Dataset* holdout)
{
  size_t desired_cars_per_dataset = 3;
  size_t desired_noncars_per_dataset = 15;
  size_t desired_cars = desired_cars_per_dataset * 2;
  size_t desired_noncars = desired_noncars_per_dataset * 2;
  ROS_ASSERT(seed.numTracks("car") >= (int)desired_cars);
  ROS_ASSERT(seed.numTracks(-1) >= (int)desired_noncars);
  vector< vector<VectorXf> > cars;
  vector< vector<VectorXf> > noncars;
  vector<VectorXf> track;
  int carid = seed.nameMapping("cmap").toId("car");
  for(int i = 0; i < seed.track_end_flags_.rows(); ++i) {
    track.push_back(seed.descriptors_.col(i));
    if(seed.track_end_flags_[i] == 1) {
      if(seed.labels_[i] == carid)
	cars.push_back(track);
      else
	noncars.push_back(track);
      track.clear();
    }
  }

  random_shuffle(cars.begin(), cars.end());
  random_shuffle(noncars.begin(), noncars.end());
  
  vector< vector<VectorXf> > cars_train;
  cars_train.insert(cars_train.begin(), cars.begin(), cars.begin() + desired_cars_per_dataset);
  vector< vector<VectorXf> > cars_holdout;
  cars_holdout.insert(cars_holdout.begin(), cars.begin() + desired_cars_per_dataset, cars.begin() + desired_cars);
  vector< vector<VectorXf> > noncars_train;
  noncars_train.insert(noncars_train.begin(), noncars.begin(), noncars.begin() + desired_noncars_per_dataset);
  vector< vector<VectorXf> > noncars_holdout;
  noncars_holdout.insert(noncars_holdout.begin(), noncars.begin() + desired_noncars_per_dataset, noncars.begin() + desired_noncars);
  
  train->applyNameMappings(seed);
  holdout->applyNameMappings(seed);
  makeDataset(cars_train, noncars_train, train);
  makeDataset(cars_holdout, noncars_holdout, holdout);
}

void runExperiment(const Dataset& seed, const Dataset& test, std::string path)
{
  ROS_ASSERT(!bfs::exists(path));
  bfs::create_directory(path);
  
  // -- Randomly draw num cars and num noncars from the seed set, without replacement.
  //    Split them between a training set and a holdout set.
  Dataset train;
  Dataset holdout;
  splitSeed(seed, &train, &holdout);
  cout << "Training set: " << endl;
  cout << train.status("  ") << endl;
  cout << "Holdout set: " << endl;
  cout << holdout.status("  ");

  // -- Train a classifier on the training set, using the training buffer.
  TrainingBuffer buffer;
  buffer.applyNameMappings(train);
  buffer.init(1e5);
  buffer.merge(train);

  ProjectionSlicer::Ptr classifier(new ProjectionSlicer);
  // We can assume that a large amount of unlabeled data is available to set good bounds on the projections.
  classifier->initialize(500, test);
  LogisticStochasticTrainer::Ptr trainer(new LogisticStochasticTrainer);
  classifier->setTrainer(trainer);
  classifier->train(*buffer.getDataset());
  
  // -- Test the classifier on the holdout set. Save accuracy vs confidence plot.
  evaluate(classifier, holdout, path + "/holdout_results");
  
  // -- Test the classifier on the full testing set. Save accuracy vs confidence plot.
  evaluate(classifier, test, path + "/full_test_results");
}

int main(int argc, char** argv)
{
  // -- Parse args.
  vector<string> seed_paths;
  vector<string> testing_paths;
  string output_path;
  
  bpo::options_description opts_desc("Allowed options");
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("test", bpo::value< vector<string> >(&testing_paths), "testing data")
    ("seed", bpo::value< vector<string> >(&seed_paths), "seed data")
    ("output", bpo::value<string>(&output_path), "output path")
    ;

  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).run(), opts);
  bpo::notify(opts);

  if(opts.count("help") || seed_paths.empty() ||
     testing_paths.empty() || output_path.empty())
  {
    cout << opts_desc << endl;
    return 1;
  }

  ROS_ASSERT(!bfs::exists(output_path));
  bfs::create_directory(output_path);
  
  cout << "Seed datasets: " << endl;
  for(size_t i = 0; i < seed_paths.size(); ++i)
    cout << "  " << seed_paths[i] << endl;
  cout << "Testing datasets: " << endl;
  for(size_t i = 0; i < testing_paths.size(); ++i)
    cout << "  " << testing_paths[i] << endl;
  cout << "Saving results to " << output_path << endl;

  // -- Load test set to be used for all experiments.
  Dataset test;
  for(size_t i = 0; i < testing_paths.size(); ++i) {
    Dataset tmp;
    cout << "Loading test dataset " << testing_paths[i] << endl;
    tmp.load(testing_paths[i]);
    test += tmp;
  }
  cout << "Test dataset status: " << endl;
  cout << test.status("  ") << endl;

  // -- Run experiments for each seed dataset.
  for(size_t i = 0; i < seed_paths.size(); ++i) {
    Dataset seed;
    cout << "Loading seed dataset " << seed_paths[i] << endl;
    seed.load(seed_paths[i]);
    cout << "Seed dataset status: " << endl;
    cout << seed.status("  ") << endl;
    string filename = seed_paths[i].substr(seed_paths[i].find_last_of("/"));
    string path = output_path + "/" + filename.substr(0, filename.find_last_of("."));
    cout << "Saving experimental results for this seed dataset to " << path << endl;
    runExperiment(seed, test, path);
  }

  return 0;
}

