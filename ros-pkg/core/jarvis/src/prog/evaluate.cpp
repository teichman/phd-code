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
    ("num-threads,j", bpo::value(&num_threads)->default_value(1))
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

  // -- Run the evaluation.
  GridClassifier::Ptr gc;
  Evaluator::Ptr ev;
  evaluateConfig(config, num_threads, train, test, &gc, &ev);

  // -- Save the results.
  ev->plot_ = false;
  ev->saveResults(output_dir);
  gc->save(output_dir + "/classifier.gc");
  
  return 0;
}
