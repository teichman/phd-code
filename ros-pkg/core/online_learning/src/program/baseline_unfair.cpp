#include <online_learning/tbssl.h>
#include <boost/program_options.hpp>
#include <boost/foreach.hpp>

using namespace std;
using namespace Eigen;
namespace bpo = boost::program_options;
namespace bfs = boost::filesystem;


void appendAnnotations(std::string path, const bpo::variables_map& opts, vector<string>* ann_paths)
{
  cout << "Checking " << path << endl;
  bfs::directory_iterator it(path), eod;
  BOOST_FOREACH(const bfs::path& p, make_pair(it, eod)) {
    if(bfs::exists(p) && bfs::extension(p).compare(".td") == 0 && p.filename().string().substr(0, string("annotated").size()).compare("annotated") == 0) {
      cout << "Adding " << p.string() << endl;
      ann_paths->push_back(p.string());
    }
  }
}

int main(int argc, char** argv)
{
  // -- Parse args.
  bpo::options_description opts_desc("Allowed options");

  vector<string> test_paths;
  int num_iters;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("root", bpo::value<string>()->default_value("."), "TBSSL root path")
    ("test", bpo::value< vector<string> >(&test_paths)->required()->multitoken(), "Test data paths.")
    ("bg", bpo::value<string>(), "Automatically labeled BG dir.  Contains training .td files.")
    ("output,o", bpo::value<string>()->required(), "Output path.")
    ("num-iters", bpo::value<int>(&num_iters)->required(), "Number of times to (accumulatively) train the classifier.")
    ;

  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).run(), opts);
  if(opts.count("help")) {
    cout << opts_desc << endl;
    return 1;
  }
  bpo::notify(opts);

  string output_path = opts["output"].as<string>();
  ROS_ASSERT(!bfs::exists(output_path));
  cout << "Saving to " << output_path << endl;

  
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
  string gc_path = iter_paths.back() + "/classifier.gc";
  gc->load(gc_path);
  gc->setZero();
  
  // -- Load annotations data.
  vector<string> ann_paths;
  for(size_t i = 0; i < iter_paths.size(); ++i) {
    string path = iter_paths[i];
    appendAnnotations(path, opts, &ann_paths);
  }
  ROS_ASSERT(!ann_paths.empty());
  TrackDataset::Ptr annotations = loadDatasets(ann_paths);
  cout << "Annotations data: " << endl;
  cout << annotations->status("  ") << endl;
  
  // -- Load test data.
  ROS_ASSERT(!test_paths.empty());
  TrackDataset::Ptr test = loadDatasets(test_paths);
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
