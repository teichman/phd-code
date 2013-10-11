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

  vector<string> train_paths;
  vector<string> test_paths;
  string output_dir;
  int num_threads;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("config", bpo::value<string>())
    ("train", bpo::value< vector<string> >(&train_paths)->required()->multitoken(), "training data")
    ("test", bpo::value< vector<string> >(&test_paths)->required()->multitoken(), "testing data")
    ("output-dir,o", bpo::value<string>(&output_dir)->required(), "Where to save results")
    ("nt", bpo::value(&num_threads)->default_value(1))
    ;

  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).run(), opts);
  if(opts.count("help")) {
    cout << opts_desc << endl;
    return 1;
  }
  bpo::notify(opts);

  // -- Create output directory.
  cout << "Saving results to " << output_dir << endl;
  if(!bfs::exists(output_dir))
    bfs::create_directory(output_dir);

  // -- Set up the initial config and save to the output dir.
  string config_path;
  if(opts.count("config"))
    config_path = opts["config"].as<string>();
  else
    config_path = ros::package::getPath("jarvis") + "/config/default_config.yml";
  cout << "Using config: " << config_path << endl;
  YAML::Node config = YAML::LoadFile(config_path);
  saveYAML(config, output_dir + "/config.yml");
  
  // -- Load data.
  cout << "Loading data." << endl;
  TrackDataset::Ptr train = loadDatasets(train_paths);
  TrackDataset::Ptr test = loadDatasets(test_paths);
  
  // -- Update descriptors on the datasets.
  cout << "Updating descriptors." << endl;
  updateDescriptors(config["Pipeline"], num_threads, train.get());
  updateDescriptors(config["Pipeline"], num_threads, test.get());
  cout << "Training set: " << endl;
  cout << train->status("  ", true) << endl;
  cout << "Testing set: " << endl;
  cout << test->status("  ", true) << endl;

  // -- Initialize the classifier
  cout << "Initializing classifier." << endl;
  srand(time(NULL));
  GridClassifier::Ptr gc(new GridClassifier);
  string ncstr = config["GlobalParams"]["NumCells"].as<string>();
  istringstream iss(ncstr);
  vector<size_t> nc;
  while(!iss.eof()) {
    size_t buf;
    iss >> buf;
    nc.push_back(buf);
    cout << "NC: " << buf << endl;
  }
  gc->initialize(*train, nc);

  // -- Train.
  cout << "Training." << endl;
  GridClassifier::BoostingTrainer::Ptr trainer(new GridClassifier::BoostingTrainer(gc));
  trainer->obj_thresh_ = config["GlobalParams"]["ObjThresh"].as<double>();
  trainer->train(train);
  gc->save(output_dir + "/classifier.gc");

  // -- Evaluate.
  cout << "Evaluating." << endl;
  Evaluator ev(gc);
  ev.evaluateParallel(*test);
  ev.plot_ = false;
  ev.saveResults(output_dir);
  
  return 0;
}
