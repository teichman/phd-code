#include <eigen_extensions/eigen_extensions.h>
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

void quantityEvaluation(const bpo::variables_map& opts, const TrackDataset& train, const TrackDataset& test,
                      const GridClassifier& base_classifier, const YAML::Node& config)
{
  cout << "************************************************************" << endl;
  cout << "* Quantity evaluation " << endl;
  cout << "************************************************************" << endl;
  
  ROS_ASSERT(train.nameMappingsAreEqual(test));
  NameMapping cmap = train.nameMapping("cmap");
  
  std::string quantity_dir = opts["output-dir"].as<string>() + "/quantity_experiment";
  if(!bfs::exists(quantity_dir))
    bfs::create_directory(quantity_dir);

  // Determine the maximum number of training examples we can ask for from the training set.
  // The limit is because we want training datasets that have the same ratio as in the test
  // set.
  size_t cidx = 0;
  size_t ptr = 0;
  size_t ntr = 0;
  for(size_t i = 0; i < train.size(); ++i) {
    if(train.label(i)(cidx) > 0)
      ++ptr;
    else if(train.label(i)(cidx) < 0)
      ++ntr;
  }
  double test_ratio = test.labelRatio(cmap.toName(cidx));
  size_t pmax = ntr * test_ratio;
  size_t nmax = ptr / test_ratio;

  size_t max_num_training_examples = 0;
  if(pmax <= ptr) {
    max_num_training_examples = pmax + ntr;
  }
  else {
    ROS_ASSERT(nmax <= ntr);
    max_num_training_examples = nmax + ptr;
  }

  // Main loop.
  size_t num_subsamples = opts["num-quantity-subsamples"].as<size_t>();
  for(size_t i = 0; i < num_subsamples; ++i) {
    // -- Create the output directory for this subsample.
    ostringstream oss;
    oss << quantity_dir << "/subsample" << setw(3) << setfill('0') << i;
    string subsample_dir = oss.str();
    bfs::create_directory(subsample_dir);
    cout << " *** " << subsample_dir << endl;
    
    size_t num_quantity_runs = opts["num-quantity-runs"].as<size_t>();
    for(size_t j = 0; j < num_quantity_runs; ++j) {
      // -- Create the output directory for this run.
      ostringstream oss;
      oss << subsample_dir << "/run" << setw(3) << setfill('0') << j;
      string run_dir = oss.str();
      bfs::create_directory(run_dir);
            
      // Generate a new training set subsample of the correct size.
      double pct = (double)(i+1) / (double)num_subsamples;
      size_t num_tracks = pct * max_num_training_examples;
      TrackDataset::Ptr training_subsample(new TrackDataset);
      *training_subsample = sampleDatasetProportional(train, num_tracks, cmap.toName(0), test.labelRatio(cmap.toName(0)));
      cout << "Training subsample:" << endl;
      cout << training_subsample->status("  ") << endl;
      cout << "Training set ratio: " << training_subsample->labelRatio(cmap.toName(0)) << endl;
      cout << "Test set ratio: " << test.labelRatio(cmap.toName(0)) << endl;

      ofstream f((run_dir + "/training_stats.txt").c_str());
      f << training_subsample->status() << endl;
      f.close();
      
      // Train a classifier.
      GridClassifier::Ptr gc(new GridClassifier(base_classifier));
      GridClassifier::BoostingTrainer::Ptr trainer(new GridClassifier::BoostingTrainer(gc));
      trainer->obj_thresh_ = config["GlobalParams"]["ObjThresh"].as<double>();
      vector<TrackDataset::ConstPtr> datasets; datasets.push_back(training_subsample);
      vector<Indices> indices; indices.push_back(Indices::All(training_subsample->size()));
      trainer->train(datasets, indices);

      // Evaluate.
      Evaluator ev(gc);
      ev.plot_ = false;
      ev.evaluateParallel(test);
      ev.saveResults(run_dir);
      cout << ev.track_stats_.statString() << endl;
    }
  }
}

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
    ("up,u", bpo::value(&up_path), "")
    ("class-names", bpo::value(&class_names)->required()->multitoken(), "")
    ("train", bpo::value(&train_paths)->required()->multitoken(), "Training data")
    ("test", bpo::value(&test_paths)->required()->multitoken(), "Testing data")
    ("num-runs", bpo::value(&num_runs)->required(), "Number of times to repeat experiment")
    ("subsample", bpo::value(&subsample)->required(), "Randomly sample this many training tracks for each run.")
    ("output-dir,o", bpo::value(&output_dir)->required(), "Where to save results")
    ("num-threads,j", bpo::value(&num_threads)->default_value(1))
    ("num-quantity-runs", bpo::value<size_t>()->default_value(10), "Number of times to run accuracy vs num tr ex plots.")
    ("num-quantity-subsamples", bpo::value<size_t>()->default_value(10), "Break the training set up into this number of subsamples for quantity evaluation.")
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
  // If using less_gravity.yml, you need to put something here, but
  // it won't have any effect.  TODO: Add accelerometer, get rid of the annoying
  // cruft involving manually setting the up vector.
  VectorXf up = VectorXf::Ones(3);
  if(opts.count("up")) {
    cout << "Setting up vector to that found at " << up_path << endl;
    eigen_extensions::loadASCII(up_path, &up);
  }

  // -- Set up the class map to use. 
  NameMapping cmap;
  cmap.addNames(class_names);
  cout << "Using cmap: " << endl;
  cout << cmap.status("  ") << endl;
  if(cmap.size() != 1) {
    ROS_FATAL("naive_supervised_baseline can only work with exactly one class at a time because of the proportional dataset splitting.");
    return 1;
  }

  // -- Load data.
  cout << "Loading data." << endl;
  TrackDataset train = *loadDatasets(train_paths, config, cmap, up, false);
  train.clearRaw();
  TrackDataset test = *loadDatasets(test_paths, config, cmap, up, false);
  test.clearRaw();

  // -- Create output directory.
  cout << "Saving results to " << output_dir << endl;
  if(!bfs::exists(output_dir))
    bfs::create_directory(output_dir);
  saveYAML(config, output_dir + "/config.yml");

  // -- Initialize the base classifier on all the data available.
  GridClassifier::Ptr base_classifier(new GridClassifier);
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
  base_classifier->initialize(*init, nc);
  init.reset();


  Evaluator ev_overall(base_classifier);
  ev_overall.plot_ = false;
  for(int i = 0; i < num_runs; ++i) {
    // -- Create the output directory for this run.
    ostringstream oss;
    oss << output_dir << "/run" << setw(3) << setfill('0') << i;
    string run_dir = oss.str();
    bfs::create_directory(run_dir);
    
    // -- Subsample the training data.
    TrackDataset ignore;
    TrackDataset::Ptr training_subsample(new TrackDataset);
    *training_subsample = sampleDatasetProportional(train, subsample, cmap.toName(0), test.labelRatio(cmap.toName(0)));
    //splitDatasetProportional(train, test.labelRatio(cmap.names()[0]), pct, training_subsample.get(), &ignore);
    cout << "Training subsample:" << endl;
    cout << training_subsample->status("  ") << endl;
    cout << "Training set ratio: " << training_subsample->labelRatio(cmap.toName(0)) << endl;
    cout << "Test set ratio: " << test.labelRatio(cmap.toName(0)) << endl;

    // -- Train the classifier.
    GridClassifier::Ptr gc(new GridClassifier(*base_classifier));
    GridClassifier::BoostingTrainer::Ptr trainer(new GridClassifier::BoostingTrainer(gc));
    trainer->obj_thresh_ = config["GlobalParams"]["ObjThresh"].as<double>();
    vector<TrackDataset::ConstPtr> datasets; datasets.push_back(training_subsample);
    vector<Indices> indices; indices.push_back(Indices::All(training_subsample->size()));
    trainer->train(datasets, indices);

    // -- Evaluate.
    Evaluator ev(gc);
    ev.plot_ = false;
    ev.evaluateParallel(test);
    ev.saveResults(run_dir);
    cout << ev.track_stats_.statString() << endl;

    ev_overall.classifier_ = gc;
    ev_overall.evaluateParallel(test);
  }

  bfs::create_directory(output_dir + "/average_results");
  ev_overall.saveResults(output_dir + "/average_results");

  
  /************************************************************
   * Evaluating on different subsets
   ************************************************************/
  quantityEvaluation(opts, train, test, *base_classifier, config);
  
  return 0;
}
