#include <boost/program_options.hpp>
#include <online_learning/active_learning_interface.h>
#include <bag_of_tricks/bag_of_tricks.h>

using namespace std;
using namespace Eigen;
namespace bpo = boost::program_options;
namespace bfs = boost::filesystem;

template<typename T>
ostream& operator<<(ostream& out, const std::vector<T>& data)
{
  for(size_t i = 0; i < data.size(); ++i)
    out << "  " << data[i] << endl;

  return out;
}

bool pathHas(std::string path, std::string ext)
{
  vector<string> files;
  bfs::recursive_directory_iterator it(path), eod;
  BOOST_FOREACH(const bfs::path& p, make_pair(it, eod)) {
    if(is_regular_file(p) && bfs::extension(p).compare(ext) == 0)
      files.push_back(p.string());
  }

  return !files.empty();
}

// This is a giant fucking mess and should be refactored into two programs,
// one for initialization and one for resumption.
int main(int argc, char** argv)
{
  // -- Set up args.
  vector<string> testing_paths;
  string unlabeled_td_dir;
  string unlabeled_tm_dir;
  size_t num_unlabeled;
  bpo::options_description opts_desc("General options, usable any time");
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("non-interactive", "no active learning")
    ("test", bpo::value< vector<string> >(&testing_paths)->required(), "testing data")
    ("unlabeled-td", bpo::value<string>(&unlabeled_td_dir)->required(), "unlabeled data (dir of td files)")
    ("unlabeled-tm", bpo::value<string>(&unlabeled_tm_dir), "unlabeled data (dir of tm files)")
    ("num-unlabeled", bpo::value<size_t>(&num_unlabeled)->default_value(2), "Number of unlabeled datasets to consider in parallel per round")
    ;
  
  vector<string> init_paths;
  vector<string> seed_paths;
  vector<string> bg_paths;
  string output_path;
  int max_iters;
  int buffer_size;
  float hand_weight;
  vector<size_t> num_cells;
  num_cells.push_back(10);
  num_cells.push_back(100);
  size_t evaluate_every;
  bpo::options_description opts_desc_init("Options only for initializing a session");
  opts_desc_init.add_options()
    ("init", bpo::value< vector<string> >(&init_paths), "init data")
    ("seed", bpo::value< vector<string> >(&seed_paths), "seed data")
    ("bg", bpo::value< vector<string> >(&bg_paths), "bg data")
    ("output", bpo::value<string>(&output_path), "output path")
    ("max-iters", bpo::value<int>(&max_iters)->default_value(0), "Maximum number of iterations")
    ("buffer-size", bpo::value<int>(&buffer_size)->default_value(5000), "Number of tracks in training buffers")
    ("hand-weight", bpo::value<float>(&hand_weight)->default_value(1), "Weight for hand-labeled training examples")
    ("num-cells", bpo::value< vector<size_t> >(&num_cells), "Default (10, 100).")
    ("evaluate-every", bpo::value<size_t>(&evaluate_every)->default_value(1), "")
    ("pos-only", "Induct positive examples only")
    ;

  string resumption_path;
  double gamma;
  bpo::options_description opts_desc_resumption("Options only for resuming a session");
  opts_desc_resumption.add_options()
    ("resume", bpo::value<string>(&resumption_path), "output path to resume.  Can provide this or --init & --output.")
    ("gamma", bpo::value<double>(&gamma)->default_value(0), "Regularization")
    ;

  bpo::options_description opts_desc_all;
  opts_desc_all.add(opts_desc).add(opts_desc_init).add(opts_desc_resumption);
  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc_all).run(), opts);
  if(opts.count("help")) {
    cout << opts_desc_all << endl;
    return 1;
  }
  bpo::notify(opts);

  // -- Make sure that resumption vs initialization params are not conflicting.
  if(opts.count("init")) {
    ROS_ASSERT(!opts.count("resume"));
    ROS_ASSERT(opts.count("seed"));
    ROS_ASSERT(opts.count("bg"));
    ROS_ASSERT(opts.count("output"));

    ROS_ASSERT(!init_paths.empty());
    ROS_ASSERT(!output_path.empty());
    ROS_ASSERT(!bfs::exists(output_path));
    bfs::create_directory(output_path);
    cout << "Using num_cells: " << endl << num_cells << endl;
  }

  if(opts.count("resume")) {
    ROS_ASSERT(!opts.count("init"));
    ROS_ASSERT(!opts.count("seed"));
    ROS_ASSERT(!opts.count("bg"));
    ROS_ASSERT(!opts.count("output"));
    ROS_ASSERT(!opts.count("num-cells"));
    ROS_ASSERT(!opts.count("pos-only"));

    ROS_ASSERT(bfs::exists(resumption_path));
  }
  
  // -- Other param checks.
  if(!opts.count("non-interactive")) {
    ROS_ASSERT(opts.count("unlabeled-tm"));
    ROS_ASSERT(bfs::exists(unlabeled_tm_dir));
    ROS_ASSERT(pathHas(unlabeled_tm_dir, ".tm"));
  }

  // -- Print what we're doing.
  if(!resumption_path.empty())
    cout << "Resuming:  " << resumption_path << endl;
  else
    cout << "Initializing a new Learner." << endl;
  
  cout << "Testing sets: " << endl << testing_paths;
  cout << "Unlabeled tm dir:  " << unlabeled_tm_dir << endl;
  cout << "Unlabeled td dir:  " << unlabeled_td_dir << endl;
  cout << "Loading testing sets and initializing learner..." << endl;
  OnlineLearner learner(loadDatasets(testing_paths));
  learner.max_iters_ = max_iters;  // If resuming, this will get overwritten.
  learner.evaluate_every_ = evaluate_every;  // If resuming, this will get overwritten.
  learner.pos_only_ = opts.count("pos-only");  // If resuming, this will get overwritten.
  learner.hand_labeled_weight_ = hand_weight;  // If resuming, this will get overwritten.
  learner.num_unl_ = num_unlabeled;  // This won't.
  learner.gamma_ = gamma;
  
  if(!resumption_path.empty()) {
    cout << "Loading snapshot from " << resumption_path << endl;
    learner.resume(resumption_path);
    if(!seed_paths.empty())
      learner.pushHandLabeledDataset(loadDatasets(seed_paths));
  }
  else {
    cout << "Initialization sets: " << endl << init_paths;
    cout << "Output dir:  " << output_path << endl;
    cout << "Seed sets: " << endl << seed_paths;
    cout << "Auto background sets: " << endl << bg_paths;

    // -- Initialize the classifier.
    cout << "Initializing classifier..." << endl;
    TrackDataset::Ptr init = loadDatasets(init_paths);
    GridClassifier::Ptr classifier(new GridClassifier);
    classifier->initialize(*init, num_cells);
    init.reset();

    TrackDataset::Ptr auto_bg(new TrackDataset);
    if(!bg_paths.empty())
      auto_bg = loadDatasets(bg_paths);
    learner.initialize(classifier, *auto_bg, output_path, unlabeled_td_dir, buffer_size, hand_weight);
    auto_bg.reset();
    if(!seed_paths.empty())
      learner.pushHandLabeledDataset(loadDatasets(seed_paths));
  }

  cout << "Launching threads..." << endl;
  ThreadPtr learning_thread = learner.launch();
    
  if(!opts.count("non-interactive")) {
    if(!resumption_path.empty()) {
      ROS_ASSERT(output_path.empty());
      output_path = resumption_path;
    }
    cout << "Starting active learning interface." << endl;
    ActiveLearningInterface ri(&learner, unlabeled_tm_dir);
    ri.run();
  }

  learning_thread->join();

  return 0;
}
