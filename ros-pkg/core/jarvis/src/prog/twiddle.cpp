#include <boost/program_options.hpp>
#include <jarvis/twiddler.h>
#include <jarvis/descriptor_pipeline.h>
#include <ros/package.h>

using namespace std;
using namespace Eigen;
using namespace pl;
namespace bfs = boost::filesystem;

#define NUM_THREADS (getenv("NUM_THREADS") ? atoi(getenv("NUM_THREADS")) : 1)

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  vector<string> train_paths;
  vector<string> test_paths;
  string output_dir;
  double max_hours;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("initial-config", bpo::value<string>())
    ("train", bpo::value< vector<string> >(&train_paths)->required()->multitoken(), "training data")
    ("test", bpo::value< vector<string> >(&test_paths)->required()->multitoken(), "testing data")
    ("output-dir,o", bpo::value<string>(&output_dir)->required(), "Where to save results")
    ("max-hours", bpo::value<double>(&max_hours)->default_value(0))
    ("hints", bpo::value< vector<string> >()->multitoken(), "Optional configuration hints")
    ;

  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << argv[0] << " [OPTS] --train [ TRAIN ... ] --test [ TEST ... ] -o OUTPUT_DIR" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  // -- Set up the initial config.
  string config_path;
  if(opts.count("initial-config"))
    config_path = opts["initial-config"].as<string>();
  else
    config_path = ros::package::getPath("jarvis") + "/config/default_config.yml";
  cout << "Using config: " << config_path << endl;
  YAML::Node config = YAML::LoadFile(config_path);

  // -- Test that pipeline construction works.
  DescriptorPipeline::registerPodTypes();
  {
    Pipeline dp(1);
    dp.deYAMLize(config["Pipeline"]);
    cout << dp.pod<DescriptorAggregator>()->dmap() << endl;
    ROS_ASSERT(JarvisTwiddler::isRequired(dp.pod< EntryPoint<Blob::Ptr> >()));
  }

  // -- Set up training and testing sets.
  TrackDataset::Ptr train = loadDatasets(train_paths);
  TrackDataset::Ptr test = loadDatasets(test_paths);
  cout << "Training set: " << endl;
  cout << train->status("  ", true) << endl;
  cout << "Testing set: " << endl;
  cout << test->status("  ", true) << endl;

  // -- Initialize the twiddler.
  JarvisTwiddler jt(train, test, NUM_THREADS);
  if(bfs::exists(output_dir))
    jt.load(output_dir);
  else
    jt.initialize(config, output_dir);

  // -- Add optional hints.
  if(opts.count("hints")) {
    vector<string> hint_paths = opts["hints"].as< vector<string> >();
    cout << "Got " << hint_paths.size() << " hints." << endl;
    for(size_t i = 0; i < hint_paths.size(); ++i) {
      YAML::Node config = YAML::LoadFile(hint_paths[i]);
      bfs::path path(hint_paths[i]);
      cout << "Adding config hint: " << path.filename().string() << endl;
      saveYAML(config, output_dir + "/hints/" + path.filename().string());
    }
  }

  // -- Go.
  cout << "Twiddling for " << max_hours << " hours maximum." << endl;
  jt.twiddle(max_hours);
  return 0;
}
