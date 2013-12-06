#include <online_learning/tbssl.h>
#include <jarvis/descriptor_pipeline.h>
#include <bag_of_tricks/glob.h>
#include <boost/program_options.hpp>
#include <boost/foreach.hpp>

using namespace std;
using namespace Eigen;
namespace bpo = boost::program_options;
namespace bfs = boost::filesystem;

void appendAnnotations(std::string dir, const bpo::variables_map& opts, vector<string>* ann_paths)
{
  cout << "Checking " << dir << endl;
  bfs::directory_iterator it(dir), eod;
  BOOST_FOREACH(const bfs::path& p, make_pair(it, eod)) {
    if(!bfs::exists(p))
      continue;
    if(bfs::extension(p).compare(".td") != 0)
      continue;
    if(p.filename().string().substr(0, string("annotated").size()).compare("annotated") != 0)
      continue;
    
    cout << "Adding " << p.string() << endl;
    ann_paths->push_back(p.string());
  }
}

int main(int argc, char** argv)
{
  // -- Parse args.
  bpo::options_description opts_desc("Allowed options");

  string up_path;
  string config_path;
  vector<string> test_paths;
  int num_iters;
  vector<string> class_names;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("up,u", bpo::value(&up_path), "")
    ("config", bpo::value(&config_path)->required(), "")
    ("root", bpo::value<string>()->default_value("."), "TBSSL root path")
    ("test", bpo::value< vector<string> >(&test_paths)->required()->multitoken(), "Test data paths.")
    ("bg", bpo::value<string>(), "Automatically labeled BG dir.  Contains training .td files.")
    ("output,o", bpo::value<string>()->required(), "Output path.")
    ("num-iters", bpo::value<int>(&num_iters)->required(), "Number of times to (accumulatively) train the classifier.")
    ("class-names", bpo::value(&class_names)->required()->multitoken(), "")
    ;

  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).run(), opts);
  if(opts.count("help")) {
    cout << opts_desc << endl;
    return 1;
  }
  bpo::notify(opts);

  string output_path = opts["output"].as<string>();
  cout << "Saving to " << output_path << endl;

  // -- Load the config.
  YAML::Node config = YAML::LoadFile(config_path);
  ROS_ASSERT(config["Pipeline"]);
  ROS_ASSERT(config["GlobalParams"]);
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
  
  // -- Get contents of root path in order.
  vector<string> iter_paths;
  bfs::directory_iterator it(opts["root"].as<string>()), eod;
  BOOST_FOREACH(const bfs::path& p, make_pair(it, eod)) {
    if(is_directory(p))
      iter_paths.push_back(p.string());
  }
  sort(iter_paths.begin(), iter_paths.end());

  // -- Load the most recent classifier and set its responses to zero.
  //    This ensures that the initialization & num cells are identical.
  GridClassifier::Ptr gc(new GridClassifier);
  bool loaded = false;
  for(int i = iter_paths.size() - 1; i >= 0; --i) {
    string gc_path = iter_paths[i] + "/classifier.gc";
    if(!bfs::exists(gc_path))
      continue;
    loaded = true;
    cout << "Loading " << gc_path << endl;
    gc->load(gc_path);
    break;
  }
  ROS_ASSERT(loaded);
  gc->setZero();
  gc->applyNameMapping("cmap", cmap);

  // -- Get the up vector.
  VectorXf up;
  if(opts.count("up")) {
    cout << "Setting up vector to that found at " << up_path << endl;
    eigen_extensions::loadASCII(up_path, &up);
    cout << "Up: " << up.transpose() << endl;
  }
  
  // -- Load annotations data.
  vector<string> ann_paths;
  for(size_t i = 0; i < iter_paths.size(); ++i) {
    //appendAnnotations(path, opts, &ann_paths);
    //ROS_WARN_ONCE("Only using ann*pos.td annotations files.  This assumes mutual exclusion.");
    ROS_WARN_ONCE("Only ann*.td annotations files.  This assumes no mutual exclusion.");
    vector<string> paths = glob(iter_paths[i] + "/ann*.td");
    ann_paths.insert(ann_paths.end(), paths.begin(), paths.end());
  }
  ROS_ASSERT(!ann_paths.empty());
  TrackDataset::Ptr annotations = loadDatasets(ann_paths, config, cmap, up);
  cout << "Annotations data: " << endl;
  cout << annotations->status("  ") << endl;
  
  // -- Load test data.
  ROS_ASSERT(!test_paths.empty());
  TrackDataset::Ptr test = loadDatasets(test_paths, config, cmap, up);
  // TODO: Clear the raw data.  We won't be needing it, and it takes up a lot of RAM.
  cout << "Test data: " << endl;
  cout << test->status("  ") << endl;

  // -- Load all bg data.
  vector<TrackDataset::ConstPtr> bg;
  if(opts.count("bg")) {
    bfs::directory_iterator it(opts["bg"].as<string>()), eod;
    BOOST_FOREACH(const bfs::path& p, make_pair(it, eod)) {
      if(bfs::exists(p) && bfs::extension(p).compare(".td") == 0) {
	cout << p.string() << ": " << endl;
	TrackDataset::Ptr td(new TrackDataset);
	td->load(p.string());
	bg.push_back(td);
	cout << td->status("  ") << endl;
      }
    }
    cout << "Loaded " << bg.size() << " auto bg datasets." << endl;
  }
  
  // -- Train.
  GridClassifier::BoostingTrainer trainer(gc);
  trainer.gamma_ = 0;
  trainer.verbose_ = true;
   //GridClassifier::StochasticLogisticTrainer trainer(gc);
  if(!bfs::exists(output_path))
    bfs::create_directory(output_path);
  
  for(int i = 0; i < num_iters; ++i) {
    cout << "Running iter " << i << endl;
    ostringstream oss;
    oss << output_path << "/iter" << setw(3) << setfill('0') << i;
    bfs::create_directory(oss.str());

    cout << "Training" << endl;
    vector<TrackDataset::ConstPtr> datasets = bg;
    datasets.push_back(annotations);
    vector<Indices> indices;
    for(size_t j = 0; j < datasets.size(); ++j)
      indices.push_back(Indices::All(datasets[j]->size()));
    trainer.train(datasets, indices);

    cout << "Evaluating" << endl;
    Evaluator ev(gc);
    ev.evaluateParallel(*test);
    ev.plot_ = false;
    ev.saveResults(oss.str());
  }
  
  return 0;
}
