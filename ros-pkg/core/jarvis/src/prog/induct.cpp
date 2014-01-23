#include <online_learning/track_dataset_visualizer.h>
#include <boost/program_options.hpp>
#include <bag_of_tricks/bag_of_tricks.h>
#include <jarvis/inductor.h>
#include <jarvis/induction_supervisor.h>
#include <jarvis/blob_view.h>
#include <jarvis/descriptor_pipeline.h>

using namespace std;
using namespace Eigen;
namespace bpo = boost::program_options;
namespace bfs = boost::filesystem;

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  // Inductor object params.
  string up_path;
  string config_path;
  double emax;
  size_t buffer_size;
  size_t max_track_length;
  int max_iters;
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

  string fake_supervisor_path;
  string fake_supervisor_config_path;
  int fake_supervisor_annotation_limit;

  vector<string> class_names;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("up,u", bpo::value(&up_path), "")
    ("config", bpo::value(&config_path)->required(), "")
    ("emax", bpo::value<double>(&emax)->required())
    ("buffer-size", bpo::value<size_t>(&buffer_size)->required())
    ("max-track-length", bpo::value<size_t>(&max_track_length)->required())
    ("max-iters", bpo::value<int>(&max_iters)->default_value(0))
    ("snapshot-every", bpo::value<int>(&snapshot_every)->required())
    ("evaluate-every", bpo::value<int>(&evaluate_every)->required())
    ("output-dir", bpo::value<string>(&output_dir)->required(), "Directory to put output.")
    ("unlabeled-td-dir", bpo::value<string>(&unlabeled_td_dir)->required(), "Directory of .td files to use as unlabeled data.")
    ("saved-annotations-dir", bpo::value<string>(&saved_annotations_dir), "Output directory of previous runs whose annotated data sequence you want to replicate.")
    
    ("test", bpo::value< vector<string> >(&test_paths)->multitoken(), ".td files to test on periodically.")
    ("seed", bpo::value< vector<string> >(&seed_paths)->multitoken(), ".td files to use as annotated seed data.")
    ("autobg", bpo::value< vector<string> >(&autobg_paths)->multitoken(), ".td files to use as automatically-annotated bg data.")

    ("init", bpo::value< vector<string> >(&init_paths)->required()->multitoken(), ".td files to initialize the classifier grids with.")
    ("class-names", bpo::value(&class_names)->required()->multitoken(), "")
    ("no-vis", "")
    ("randomize", "Set the random seed to a random number.")
    ("fake-supervisor", bpo::value(&fake_supervisor_path), "Path to GridClassifier")
    ("fake-supervisor-config", bpo::value(&fake_supervisor_config_path), "Path to config used by fake supervisor")
    ("fake-supervisor-annotation-limit", bpo::value(&fake_supervisor_annotation_limit)->default_value(-1), "FakeSupervisor will only provide corrections while OnlineLearner has fewer than this many annotations")
    ("active-learning", "Use active learning rather than group induction")
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

  // -- Get the up vector.
  VectorXf up;
  if(opts.count("up")) {
    cout << "Setting up vector to that found at " << up_path << endl;
    eigen_extensions::loadASCII(up_path, &up);
    cout << "Up: " << up.transpose() << endl;
  }
  
  // -- Initialize classifier and trainer.
  cout << "Loading initialization datasets..." << endl;
  TrackDataset::Ptr init = loadDatasets(init_paths, config, cmap, up, true);
  cout << "Initializing classifier..." << endl;
  GridClassifier::Ptr classifier(new GridClassifier);
  classifier->initialize(*init, nc);
  ROS_ASSERT(classifier->nameMappingsAreEqual(*init));
  cout << "dmap: " << classifier->nameMapping("dmap") << endl;
  init.reset();
  
  GridClassifier::BoostingTrainer::Ptr trainer(new GridClassifier::BoostingTrainer(classifier));
  trainer->verbose_ = true;
  trainer->gamma_ = 0;
  trainer->obj_thresh_ = config["GlobalParams"]["ObjThresh"].as<double>();
  trainer->applyNameMappings(*classifier);
  cout << "trainer->obj_thresh_: " << trainer->obj_thresh_ << endl;
    
  // -- Initialize Inductor.
  cout << "Initializing Inductor..." << endl;
  Inductor inductor(config, emax, buffer_size, max_track_length,
                    classifier, trainer, max_iters, snapshot_every,
                    evaluate_every, output_dir, unlabeled_td_dir,
                    saved_annotations_dir);
  inductor.up_ = up;

  if(opts.count("active-learning")) {
    ROS_WARN("Using active learning rather than group induction.");
    inductor.active_learning_ = true;
  }

  if(!seed_paths.empty()) {
    TrackDataset::Ptr seed = loadDatasets(seed_paths, config, cmap, up, true);
    for(size_t i = 0; i < seed->size(); ++i) {
      const Dataset& track = *seed->tracks_[i];
      for(size_t j = 0; j < track.size(); ++j) {
        ROS_ASSERT(!track[j].raw_.empty());
      }
    }
    cout << "Using seed dataset: " << endl;
    cout << seed->status("  ");
    inductor.pushHandLabeledDataset(seed);
  }

  if(!autobg_paths.empty()) {
    TrackDataset::Ptr autobg = loadDatasets(autobg_paths, config, cmap, up, true);
    for(size_t i = 0; i < autobg->size(); ++i) {
      const Dataset& track = *autobg->tracks_[i];
      for(size_t j = 0; j < track.size(); ++j) {
        ROS_ASSERT(!track[j].raw_.empty());
      }
    }
    cout << "Using autobg dataset: " << endl;
    cout << autobg->status("  ");
    inductor.pushAutoLabeledDataset(autobg);
  }

  if(!test_paths.empty()) {
    TrackDataset::Ptr test = loadDatasets(test_paths, config, cmap, up, true);
    inductor.setTestData(test);
    cout << "Using test dataset: " << endl;
    cout << test->status("  ");
  }

  // -- Set up induction supervisor.
  InductionSupervisor::Ptr isup;
  if(opts.count("fake-supervisor")) {
    ROS_ASSERT(opts.count("fake-supervisor-config"));
    cout << "Using fake supervisor at " << fake_supervisor_path << endl;
    cout << "  Config: " << fake_supervisor_config_path << endl;
    cout << "  Annotation limit: " << fake_supervisor_annotation_limit << endl;

    // If we're using a fake supervisor annotation limit,
    // ignore the max iters option.  Use the annotation limit to decide when
    // to stop instead.
    if(fake_supervisor_annotation_limit != -1) {
      inductor.max_annotations_ = fake_supervisor_annotation_limit;
      inductor.setMaxIters(-1);
    }
    
    GridClassifier gc;
    gc.load(fake_supervisor_path);
    YAML::Node fs_config = YAML::LoadFile(fake_supervisor_config_path);
    ROS_ASSERT(fs_config["Pipeline"]);
    isup = InductionSupervisor::Ptr(new InductionSupervisor(gc, fs_config, up, &inductor, 0.5, output_dir));
    isup->annotation_limit_ = fake_supervisor_annotation_limit;
    isup->launch();
  }
  
  // -- Go.
  ThreadPtr learning_thread = inductor.launch();

  if(opts.count("no-vis"))
    learning_thread->join();
  else {
    BlobView view;
    VCMultiplexor multiplexor(&view);
    ActiveLearningViewController alvc(&multiplexor, &inductor, unlabeled_td_dir);
    InductionViewController ivc(&inductor, &multiplexor);
    multiplexor.addVC(&alvc);
    multiplexor.addVC(&ivc);
    
    ThreadPtr view_thread = view.launch();
    ThreadPtr alvc_thread = alvc.launch();
    ThreadPtr ivc_thread = ivc.launch();

    learning_thread->join();
    
    view.quit();
    alvc.quit();
    ivc.quit();
    view_thread->join();
    alvc_thread->join();
    ivc_thread->join();
  }

  if(isup) {
    isup->quit();
    isup->thread()->join();
  }

  return 0;
}
