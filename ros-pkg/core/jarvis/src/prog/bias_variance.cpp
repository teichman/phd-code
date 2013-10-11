#include <matplotlib_interface/matplotlib_interface.h>  // Must come first because of Python.
#include <boost/program_options.hpp>
#include <boost/foreach.hpp>
#include <online_learning/tbssl.h>
#include <performance_statistics/performance_statistics.h>
#include <jarvis/descriptor_pipeline.h>

using namespace std;
using namespace Eigen;
namespace bpo = boost::program_options;
namespace bfs = boost::filesystem;


void run(string runpath,
         YAML::Node config,
	 const GridClassifier& gcbase,
	 vector<TrackDataset::Ptr> train,
	 const TrackDataset& test,
	 bpo::variables_map opts,
	 vector<double>* final_accs,
	 vector<double>* num_tracks)
{
  for(size_t i = 0; i < train.size(); ++i)
    ROS_ASSERT(train[i]->nameMappingsAreEqual(test));
  
  // -- Randomize the ordering of the TDs (but not the tracks within the TDs).
  random_shuffle(train.begin(), train.end());

  // -- Get the proportions of each class in the training set.
  //    This is only valid if all classes are mutually exclusive.
  VectorXd total_per_class = VectorXd::Zero(gcbase.nameMapping("cmap").size());
  double total_annotated = 0;
  double total_bg = 0;
  for(size_t i = 0; i < train.size(); ++i) {
    const TrackDataset& td = *train[i];
    for(size_t j = 0; j < td.size(); ++j) {
      if(td[j].size() == 0)
	continue;
      
      int id = td.label(j).id();
      if(id < -1)
	continue;
      
      ++total_annotated;
      if(id == -1)
	++total_bg;
      else
	++total_per_class(id);
    }
  }
  double bg_proportion = total_bg / total_annotated;
  cout << "Total bg: " << total_bg << endl;
  cout << "Bg proportion: " << bg_proportion << endl;
  VectorXd proportions = total_per_class / total_annotated;
  cout << "Per-class totals: " << total_per_class.transpose() << endl;
  cout << "Per-class proportions: " << proportions.transpose() << endl;
  ROS_ASSERT(fabs(proportions.sum() + bg_proportion - 1.0) < 1e-6);
  
  // -- Main loop.
  vector<double> pcts = opts["pcts"].as< vector<double> >();
  for(size_t p = 0; p < pcts.size(); ++p) {
    double sz = pcts[p];
    size_t num = sz * total_annotated;
    ostringstream oss;
    oss << runpath << "/" << setw(7) << setfill('0') << num << "tracks";
    string part_path = oss.str();
    bfs::create_directory(part_path);

    VectorXi desired_per_class(gcbase.nameMapping("cmap").size());
    desired_per_class.setZero();
    desired_per_class = (proportions * num).cast<int>();
    int desired_bg = bg_proportion * num;
    cout << "Desired bg: " << desired_bg << endl;
    cout << "Desired per-class: " << desired_per_class.transpose() << endl;
    
    // -- Get a partial dataset.
    TrackDataset::Ptr td(new TrackDataset);
    td->applyNameMappings(test);
    td->tracks_.reserve(num);

    int curr_bg = 0;
    VectorXi curr_per_class = VectorXi::Zero(gcbase.nameMapping("cmap").size());
    for(size_t i = 0; i < train.size(); ++i) {
      const TrackDataset& ttd = *train[i];
      for(size_t j = 0; j < ttd.size(); ++j) {
	if(ttd.size() == 0)
	  continue;
	int id = ttd.label(j).id();
	if(id < -1)
	  continue;

	if(id == -1 && curr_bg < desired_bg) {
	  td->tracks_.push_back(ttd.tracks_[j]);
	  ++curr_bg;
	}
	else if(id > -1 && curr_per_class(id) < desired_per_class(id)) {
	  td->tracks_.push_back(ttd.tracks_[j]);
	  ++curr_per_class(id);
	}
      }
    }

    cout << "Generated dataset " << sz << endl;
    cout << td->status("  ") << endl;
    ofstream file;
    file.open((part_path + "/dataset_status.txt").c_str());
    file << td->status() << endl;
    file << endl;
    file << "Status of the dataset ordering: " << endl;
    for(size_t i = 0; i < train.size(); ++i) {
      file << train[i]->status() << endl;
    }
    file.close();
	
    // -- Train on it until performance stops increasing.
    GridClassifier::Ptr gc(new GridClassifier);
    *gc = gcbase;  // Re-use initialization.
    Trainer::Ptr trainer;
    GridClassifier::BoostingTrainer::Ptr bt = GridClassifier::BoostingTrainer::Ptr(new GridClassifier::BoostingTrainer(gc));
    bt->verbose_ = true;
    bt->obj_thresh_ = config["GlobalParams"]["ObjThresh"].as<double>();
    trainer = bt;

    // Datasets to train on: all bg, plus the one we just constructed.
    vector<TrackDataset::ConstPtr> datasets;
    datasets.push_back(td);
    vector<Indices> indices;
    for(size_t i = 0; i < datasets.size(); ++i)
      indices.push_back(Indices::All(datasets[i]->size()));
    
    vector<double> acc(1, 0.0);
    
    cout << "Training..." << endl;
    HighResTimer hrt("Training");
    hrt.start();
    trainer->train(datasets, indices);
    hrt.stop();
    cout << hrt.report() << endl;
    
    cout << "Evaluating..." << endl;
    hrt.reset("Evaluating");
    hrt.start();
    Evaluator ev(gc);
    ev.evaluateParallel(test);
    hrt.stop();
    cout << hrt.report() << endl;
    
    acc.push_back(ev.track_stats_.getTotalAccuracy());
    cout << "Accuracy " << acc.back() << endl;
    
    ev.plot_ = false;
    ev.saveResults(part_path);

    gc->save(part_path + "/classifier.gc");
    final_accs->push_back(acc.back());
    num_tracks->push_back(num);
    cout << "Num tracks: " << num << "  ";
    cout << "Final acc: " << acc.back() << endl;
  }
}
  
int main(int argc, char** argv)
{
  // -- Parse args.
  bpo::options_description opts_desc("Allowed options");
  string config_path;
  int num_threads;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("test", bpo::value<string>()->required(), "Test set dir.  Contains test .td files.")
    ("train", bpo::value<string>()->required(), "Training set dir.  Contains training .td files.")
    ("output,o", bpo::value<string>()->default_value("baseline"), "Output dir")
    ("num-runs,n", bpo::value<int>()->default_value(1))
    ("config", bpo::value(&config_path)->required())
    ("pcts", bpo::value< std::vector<double> >()->required()->multitoken(), "Percent of the full training set to use.  Can provide multiple, must provide at least one.  [0, 1].")
    ("num-threads,j", bpo::value(&num_threads)->default_value(1))
    ;

  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).run(), opts);
  if(opts.count("help")) {
    cout << opts_desc << endl;
    return 1;
  }
  bpo::notify(opts);

  srand(time(NULL));
  
  string output_path = opts["output"].as<string>();
  ROS_ASSERT(!bfs::exists(output_path));
  cout << "Saving results to " << output_path << endl;
  bfs::create_directory(output_path);
  
  // -- Load the config.
  cout << "Using config: " << config_path << endl;
  YAML::Node config = YAML::LoadFile(config_path);
  saveYAML(config, output_path + "/config.yml");
  string ncstr = config["GlobalParams"]["NumCells"].as<string>();
  istringstream iss(ncstr);
  vector<size_t> num_cells;
  while(!iss.eof()) {
    size_t buf;
    iss >> buf;
    num_cells.push_back(buf);
  }
      
  // -- Load all training data.
  vector<TrackDataset::Ptr> train;
  {
    bfs::recursive_directory_iterator it(opts["train"].as<string>()), eod;
    BOOST_FOREACH(const bfs::path& p, make_pair(it, eod)) {
      if(bfs::exists(p) && bfs::extension(p).compare(".td") == 0) {
	cout << p.string() << ": " << endl;
	TrackDataset::Ptr td(new TrackDataset);
	td->load(p.string());
        updateDescriptors(config["Pipeline"], num_threads, td.get());
	train.push_back(td);
	cout << td->status("  ") << endl;
      }
    }
  }
  ROS_ASSERT(!train.empty());

  // -- Initialize GC using all the training data.
  cout << "Initializing using num_cells ";
  for(size_t i = 0; i < num_cells.size(); ++i)
    cout << " " << num_cells[i];
  cout << endl;
  GridClassifier gcbase;
  TrackDataset init = *train[0];
  for(size_t i = 1; i < train.size(); ++i)
    init += *train[i];
  gcbase.initialize(init, num_cells);
  
  // -- Load test data.
  vector<string> test_paths;
  bfs::recursive_directory_iterator it(opts["test"].as<string>()), eod;
  BOOST_FOREACH(const bfs::path& p, make_pair(it, eod)) {
    if(bfs::exists(p) && bfs::extension(p).compare(".td") == 0)
      test_paths.push_back(p.string());
  }
  ROS_ASSERT(!test_paths.empty());
  TrackDataset::Ptr test = loadDatasets(test_paths);
  updateDescriptors(config["Pipeline"], num_threads, test.get());
  cout << "Test data: " << endl;
  cout << test->status("  ") << endl;

  // -- Run the test several times.
  vector< vector<double> > accs;
  vector< vector<double> > nums;
  for(int i = 0; i < opts["num-runs"].as<int>(); ++i) {
    ostringstream oss;
    oss << opts["output"].as<string>() << "/run" << setw(3) << setfill('0') << i;
    string runpath = oss.str();
    bfs::create_directory(runpath);
    
    vector<double> acc;
    vector<double> num;
    run(runpath, config, gcbase, train, *test, opts, &acc, &num);
    accs.push_back(acc);
    nums.push_back(num);
  }

  // -- Plot.
  // TODO: This is hanging on capek.  Why?
  // mpliBegin();
  // mpli("import matplotlib.pyplot as plt");
  // mpli("import numpy as np");
  // mpliPrintSize();
  // for(size_t i = 0; i < accs.size(); ++i) {
  //   mpliNamedExport("acc", accs[i]);
  //   mpliNamedExport("num", nums[i]);
  //   mpli("plt.plot(num, acc, 'r-')");
  // }
  // mpli("plt.xlabel('Number of tracks')");
  // mpli("plt.ylabel('Accuracy')");
  // mpli("plt.ylim(0, 1)");

  // mpliExport(output_path);
  // mpli("plt.savefig(output_path + '/plot.png')");
  // mpli("plt.savefig(output_path + '/plot.pdf')");
  
  return 0;
}
