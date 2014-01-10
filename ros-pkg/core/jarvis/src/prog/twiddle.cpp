#include <jarvis/jarvis_twiddler.h>
#include <boost/program_options.hpp>
#include <jarvis/descriptor_pipeline.h>
#include <ros/package.h>
#include <bag_of_tricks/glob.h>

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

  string config_path;
  string regression_test_dir;
  vector<string> names;
  string output_dir;
  double max_hours;
  vector<string> hint_paths;
  double decimate;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("initial-config", bpo::value(&config_path)->required(), "")
    ("regression-test-dir", bpo::value(&regression_test_dir)->required(), "")
    ("output-dir,o", bpo::value<string>(&output_dir)->required(), "Where to save results")
    ("regression-tests", bpo::value(&names)->multitoken(), "If provided, only use these regression tests")
    ("max-hours", bpo::value<double>(&max_hours)->default_value(0))
    ("hints", bpo::value(&hint_paths)->multitoken(), "Optional configuration hints")
    ("decimate", bpo::value(&decimate)->default_value(0), "Downsample tracks this much.  (0, 1).")
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

  // -- Set up the initial config.
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

  // We don't want decimate to get a different split every time.
  // So, set the random seed to zero.  This should be unnecessary.
  srand(0);  
  
  // -- Load data.
  vector<string> test_dirs = glob(regression_test_dir + "/*");
  vector<TrackDataset> datasets;
  vector<VectorXf> up_vectors;
  for(size_t i = 0; i < test_dirs.size(); ++i) {
    string name = bfs::path(test_dirs[i]).leaf().string();
    if(!names.empty() && find(names.begin(), names.end(), name) == names.end()) {
      cout << "Skipping test \"" << name
           << "\" because it was not specified but other tests were." << endl;
      continue;
    }
    cout << "Loading test \"" << name << "\"." << endl;

    VectorXf up;
    eigen_extensions::loadASCII(test_dirs[i] + "/up.eig.txt", &up);
    up_vectors.push_back(up);

    NameMapping cmap;
    ifstream f(test_dirs[i] + "/class_names.txt");
    while(true) {
      string name;
      f >> name;
      if(f.eof()) break;
      cmap.addName(name);
    }
    cout << cmap << endl;
    
    vector<string> td_paths = glob(test_dirs[i] + "/test/*.td");
    TrackDataset td = *loadDatasets(td_paths, config, cmap, up, true);
    if(decimate != 0) {      
      TrackDataset split0, split1;
      splitDataset(td, decimate, &split0, &split1);
      td = split1;
    }
    datasets.push_back(td);
  }
  ROS_ASSERT(!datasets.empty());

  if(opts.count("randomize")) {
    cout << "Setting random seed to something random." << endl;
    cout << "Note this does not affect decimation splitting, which always uses the same random seed." << endl;
    srand(time(NULL));
  }
  
  // -- Initialize the twiddler.
  JarvisTwiddler jt(datasets, up_vectors, NUM_THREADS);
  if(bfs::exists(output_dir))
    jt.load(output_dir);
  else
    jt.initialize(config, output_dir);

  // -- Add optional hints.
  cout << "Got " << hint_paths.size() << " hints." << endl;
  for(size_t i = 0; i < hint_paths.size(); ++i) {
    YAML::Node config = YAML::LoadFile(hint_paths[i]);
    bfs::path path(hint_paths[i]);
    cout << "Adding config hint: " << path.filename().string() << endl;
    saveYAML(config, output_dir + "/hints/" + path.filename().string());
  }

  // -- Go.
  if(max_hours > 0)
    cout << "Twiddling for " << max_hours << " hours maximum." << endl;
  jt.twiddle(max_hours);
  return 0;
}
