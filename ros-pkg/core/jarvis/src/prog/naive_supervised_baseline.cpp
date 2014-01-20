#include <jarvis/jarvis_twiddler.h>
#include <boost/program_options.hpp>
#include <ros/package.h>
#include <online_learning/evaluator.h>
#include <online_learning/grid_classifier.h>
#include <jarvis/descriptor_pipeline.h>

using namespace std;
using namespace Eigen;
namespace bpo = boost::program_options;
namespace bfs = boost::filesystem;

int main(int argc, char** argv)
{
  // -- Parse args.
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  string config_path;
  vector<string> train_paths;
  vector<string> test_paths;
  string output_dir;
  int num_threads;
  int subsample;
  int num_runs;
  string up_path;
  vector<string> class_names;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("config", bpo::value(&config_path)->required())
    ("up,u", bpo::value(&up_path)->required(), "")
    ("class-names", bpo::value(&class_names)->required()->multitoken(), "")
    ("train", bpo::value(&train_paths)->required()->multitoken(), "Training data")
    ("test", bpo::value(&test_paths)->required()->multitoken(), "Testing data")
    ("num-runs", bpo::value(&num_runs)->required(), "Number of times to repeat experiment")
    ("subsample", bpo::value(&subsample)->required(), "Randomly sample this many training tracks for each run.")
    ("output-dir,o", bpo::value(&output_dir)->required(), "Where to save results")
    ("num-threads,j", bpo::value(&num_threads)->default_value(1))
    ("randomize", "")
    ;

  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << argv[0] << " [OPTS]" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  if(opts.count("randomize")) {
    cout << "Setting the random seed to something random." << endl;
    srand(time(NULL));
  }

  // -- Make sure the output directory has not been used for a previous run.
  ROS_ASSERT(!bfs::exists(output_dir + "/run000"));
    
  // -- Set up the initial config.
  cout << "Using config: " << config_path << endl;
  YAML::Node config = YAML::LoadFile(config_path);
  
  // -- Get the up vector.
  VectorXf up;
  cout << "Setting up vector to that found at " << up_path << endl;
  eigen_extensions::loadASCII(up_path, &up);
  cout << "Up: " << up.transpose() << endl;

  // -- Set up the class map to use. 
  NameMapping cmap;
  cmap.addNames(class_names);
  cout << "Using cmap: " << endl;
  cout << cmap.status("  ") << endl;

  // -- Load data.
  cout << "Loading data." << endl;
  TrackDataset train = *loadDatasets(train_paths, config, cmap, up, false);
  TrackDataset test = *loadDatasets(test_paths, config, cmap, up, false);

  // -- Create output directory.
  cout << "Saving results to " << output_dir << endl;
  if(!bfs::exists(output_dir))
    bfs::create_directory(output_dir);
  saveYAML(config, output_dir + "/config.yml");

  // -- Initialize the base classifier on all the data available.
  GridClassifier base_classifier;
  string ncstr = config["GlobalParams"]["NumCells"].as<string>();
  istringstream iss(ncstr);
  vector<size_t> nc;
  while(!iss.eof()) {
    size_t buf;
    iss >> buf;
    nc.push_back(buf);
  }
  TrackDataset::Ptr init(new TrackDataset(train));
  *init += test;
  base_classifier.initialize(*init, nc);
  init.reset();

  
  for(int i = 0; i < num_runs; ++i) {
    // -- Create the output directory for this run.
    ostringstream oss;
    oss << output_dir << "/run" << setw(3) << setfill('0') << i;
    string run_dir = oss.str();
    bfs::create_directory(run_dir);
    
    // -- Subsample the training data.
    double pct = (double)subsample / train.size();
    TrackDataset training_subsample, ignore;
    splitDataset(train, pct, &training_subsample, &ignore);
    cout << "Training subsample:" << endl;
    cout << training_subsample.status("  ") << endl;

    // -- Train the classifier.
    GridClassifier::Ptr gc(new GridClassifier(base_classifier));
    GridClassifier::BoostingTrainer::Ptr trainer(new GridClassifier::BoostingTrainer(gc));
    trainer->obj_thresh_ = config["GlobalParams"]["ObjThresh"].as<double>();
    trainer->train(training_subsample);

    // -- Evaluate.
    Evaluator::Ptr ev(new Evaluator(gc));
    ev->plot_ = false;
    ev->evaluateParallel(test);
    ev->saveResults(run_dir);

    cout << ev->track_stats_.statString() << endl;
  }
  
  return 0;
}
