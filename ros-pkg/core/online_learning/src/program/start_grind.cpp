#include <online_learning/track_dataset_visualizer.h>

#include <boost/program_options.hpp>
#include <bag_of_tricks/bag_of_tricks.h>
#include <online_learning/tbssl.h>

using namespace std;
using namespace Eigen;
namespace bpo = boost::program_options;
namespace bfs = boost::filesystem;

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  // OnlineLearner object params.
  double emax;
  size_t buffer_size;
  size_t max_track_length;
  double gamma;
  int snapshot_every;
  int evaluate_every;
  string output_dir;
  string unlabeled_td_dir;
  string saved_annotations_dir;

  // Data to pass to OnlineLearner.
  vector<string> test_paths;
  vector<string> seed_paths;
  vector<string> autobg_paths;

  // Classifier params.
  vector<string> init_paths;
  vector<size_t> num_cells;
  num_cells.push_back(10);
  num_cells.push_back(100);

  opts_desc.add_options()
    ("help,h", "produce help message")
    ("emax", bpo::value<double>(&emax)->required())
    ("buffer-size", bpo::value<size_t>(&buffer_size)->required())
    ("max-track-length", bpo::value<size_t>(&max_track_length)->required())
    ("gamma", bpo::value<double>(&gamma)->required())
    ("snapshot-every", bpo::value<int>(&snapshot_every)->required())
    ("evaluate-every", bpo::value<int>(&evaluate_every)->required())
    ("output-dir", bpo::value<string>(&output_dir)->required(), "Directory to put output.")
    ("unlabeled-td-dir", bpo::value<string>(&unlabeled_td_dir)->required(), "Directory of .td files to use as unlabeled data.")
    ("saved-annotations-dir", bpo::value<string>(&saved_annotations_dir), "Output directory of previous runs whose annotated data sequence you want to replicate.")
    
    ("test", bpo::value< vector<string> >(&test_paths)->multitoken(), ".td files to test on periodically.")
    ("seed", bpo::value< vector<string> >(&seed_paths)->multitoken(), ".td files to use as annotated seed data.")
    ("autobg", bpo::value< vector<string> >(&autobg_paths)->multitoken(), ".td files to use as automatically-annotated bg data.")

    ("init", bpo::value< vector<string> >(&init_paths)->required()->multitoken(), ".td files to initialize the classifier grids with.")
    ("num-cells", bpo::value< vector<size_t> >(&num_cells)->multitoken(), "Default is --num-cells 10 100.")
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
  
  // -- Initialize classifier.
  cout << "Loading initialization datasets..." << endl;
  TrackDataset::Ptr init = loadDatasets(init_paths);
  cout << "Initializing classifier..." << endl;
  GridClassifier::Ptr classifier(new GridClassifier);
  classifier->initialize(*init, num_cells);
  init.reset();

  // -- Initialize OnlineLearner.
  cout << "Initializing OnlineLearner..." << endl;
  int max_iters = 0;
  OnlineLearner learner(emax, buffer_size, max_track_length, gamma,
                        classifier, max_iters, snapshot_every,
                        evaluate_every, output_dir, unlabeled_td_dir,
                        saved_annotations_dir);

  if(!seed_paths.empty()) {
    TrackDataset::Ptr seed = loadDatasets(seed_paths);
    for(size_t i = 0; i < seed->size(); ++i) {
      const Dataset& track = *seed->tracks_[i];
      for(size_t j = 0; j < track.size(); ++j) {
        ROS_ASSERT(!track[j].raw_.empty());
      }
    }
    cout << "Using seed dataset: " << endl;
    cout << seed->status("  ");
    learner.pushHandLabeledDataset(seed);
  }
  if(!autobg_paths.empty()) {
    TrackDataset::Ptr autobg = loadDatasets(autobg_paths);
    for(size_t i = 0; i < autobg->size(); ++i) {
      const Dataset& track = *autobg->tracks_[i];
      for(size_t j = 0; j < track.size(); ++j) {
        ROS_ASSERT(!track[j].raw_.empty());
      }
    }
    cout << "Using autobg dataset: " << endl;
    cout << autobg->status("  ");
    learner.pushAutoLabeledDataset(autobg);
  }

  if(!test_paths.empty()) {
    TrackDataset::Ptr test = loadDatasets(test_paths);
    learner.setTestData(test);
    cout << "Using test dataset: " << endl;
    cout << test->status("  ");
  }

  // -- Go.
  ThreadPtr learning_thread = learner.launch();

  DGCTrackView view;
  VCMultiplexor multiplexor(&view);
  ActiveLearningViewController alvc(&multiplexor, &learner, unlabeled_td_dir);
  InductionViewController ivc(&learner, &multiplexor);
  multiplexor.addVC(&alvc);
  multiplexor.addVC(&ivc);

  ThreadPtr view_thread = view.launch();
  ThreadPtr alvc_thread = alvc.launch();
  ThreadPtr ivc_thread = ivc.launch();

  learning_thread->join();
  view_thread->join();
  alvc_thread->join();
  ivc_thread->join();

  return 0;
}
