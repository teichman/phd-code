#include <boost/program_options.hpp>
#include <boost/foreach.hpp>
#include <eigen_extensions/eigen_extensions.h>
#include <jarvis/descriptor_pipeline.h>
#include <online_learning/cross_evaluator.h>

using namespace std;
using namespace Eigen;
namespace bpo = boost::program_options;
namespace bfs = boost::filesystem;

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  string config_path;
  vector<string> class_names;
  vector<string> dataset_paths;
  string output_dir;
  string up_path;
  size_t num_orderings;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("config", bpo::value(&config_path)->required(), "")
    ("class-names", bpo::value(&class_names)->required()->multitoken(), "")
    ("tds", bpo::value(&dataset_paths)->required()->multitoken(), "Labeled data")
    ("output,o", bpo::value(&output_dir)->required(), "Output directory")
    ("up,u", bpo::value(&up_path), "")
    ("vary-training-set-size", "")
    ("num-orderings", bpo::value(&num_orderings)->default_value(3), "Only makes sense with --vary-training-set-size")
    ;

  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << argv[0] << " [OPTS] TD [ TD ... ] -o OUTPUT_DIR" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  // -- Load the config.
  YAML::Node config = YAML::LoadFile(config_path);
  ROS_ASSERT(config["Pipeline"]);
  ROS_ASSERT(config["GlobalParams"]);
  double thresh = config["GlobalParams"]["ObjThresh"].as<double>();
  string ncstr = config["GlobalParams"]["NumCells"].as<string>();
  istringstream iss(ncstr);
  vector<size_t> nc;
  cout << "Num cells: ";
  while(!iss.eof()) {
    size_t buf;
    iss >> buf;
    nc.push_back(buf);
    cout << buf << " ";
  }
  cout << endl;
  
  // -- Set up the class map to use. 
  NameMapping cmap;
  cmap.addNames(class_names);
  cout << "Using cmap: " << endl;
  cout << cmap.status("  ") << endl;
  
  // -- Get dataset names.
  vector<string> dataset_names(dataset_paths.size());
  for(size_t i = 0; i < dataset_paths.size(); ++i) {
    bfs::path path(dataset_paths[i]);
    dataset_names[i] = path.stem().string();
    cout << "Name: " << dataset_names[i] << endl;
  }

  // -- Get the up vector.
  VectorXf up;
  if(opts.count("up")) {
    cout << "Setting up vector to that found at " << up_path << endl;
    eigen_extensions::loadASCII(up_path, &up);
    cout << "Up: " << up.transpose() << endl;
  }
  
  // -- Load all datasets.
  vector<TrackDataset::ConstPtr> tds;
  for(size_t i = 0; i < dataset_paths.size(); ++i) {
    cout << "Loading " << dataset_paths[i] << endl;
    TrackDataset::Ptr td(new TrackDataset);
    td->load(dataset_paths[i]);
    td->applyNameMapping("cmap", cmap);
    updateDescriptors(config["Pipeline"], 24, td.get(), up);
    tds.push_back(td);
  }

  CrossEvaluator ce;
  for(size_t i = 0; i < tds.size(); ++i)
    ce.addTrackDataset(tds[i], dataset_names[i]);

  if(opts.count("vary-training-set-size"))
    ce.evaluateTrainingSetSize(output_dir, num_orderings, nc, thresh);
  else
    ce.evaluate(output_dir, nc, thresh);

  return 0;
}
