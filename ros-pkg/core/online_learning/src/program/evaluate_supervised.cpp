#include <online_learning/tbssl.h>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#define INCREMENTAL_EVAL (getenv("INCREMENTAL_EVAL") ? atoi(getenv("INCREMENTAL_EVAL")) : 0)
#define NUM_PASSES (getenv("NUM_PASSES") ? atoi(getenv("NUM_PASSES")) : 1)
#define NUM_CELLS (getenv("NUM_CELLS") ? atoi(getenv("NUM_CELLS")) : 500)

using namespace std;
namespace po = boost::program_options;
namespace bfs = boost::filesystem;
using boost::dynamic_pointer_cast;

NameMapping g_common_class_map;
NameMapping g_common_descriptor_map;

ostream& operator<<(ostream& out, const vector<string>& strings)
{
  for(size_t i = 0; i < strings.size(); ++i)
    out << "  " << strings[i] << endl;

  return out;
}

void evaluate(const vector<string>& testing_paths,
	      int num_training_instances,
	      Classifier::ConstPtr classifier,
	      const string& path)
{
  Evaluator evaluator(classifier);
  for(size_t i = 0; i < testing_paths.size(); ++i) {
    TrackDataset::Ptr data(new TrackDataset());
    data->load(testing_paths[i]);
    cout << "Evaluating on " << testing_paths[i] << endl;

    ROS_ASSERT(g_common_class_map == data->nameMapping("cmap"));
    ROS_ASSERT(g_common_descriptor_map == data->nameMapping("dmap"));

    evaluator.evaluateParallel(*data);
  }

  bfs::create_directory(path);
  evaluator.saveResults(path);
  classifier->save(path + "/classifier.gc");
  ofstream file;
  file.open((path + "/num_training_instances.txt").c_str());
  file << num_training_instances << endl;
  file.close();
}

int main(int argc, char** argv)
{
  cout << "INCREMENTAL_EVAL: " << INCREMENTAL_EVAL << endl;
  cout << "NUM_PASSES: " << NUM_PASSES << endl;
  
  // -- Parse args.
  vector<string> init_paths;
  vector<string> training_paths;
  vector<string> testing_paths;
  string training_method;
  string output_path;
  
  po::options_description opts_desc("Allowed options");
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("method", po::value<string>(&training_method), "training method")
    ("test", po::value< vector<string> >(&testing_paths), "testing data")
    ("train", po::value< vector<string> >(&training_paths), "training data")
    ("init", po::value< vector<string> >(&init_paths), "init data")
    ("output", po::value<string>(&output_path), "output path")
    ;

  po::variables_map opts;
  po::store(po::command_line_parser(argc, argv).options(opts_desc).run(), opts);
  po::notify(opts);

  if(opts.count("help") || training_paths.empty() ||
     testing_paths.empty() || output_path.size() == 0) {
    cout << opts_desc << endl;
    return 0;
  }

  if(training_method.compare("GridClassifier") != 0) {
    ROS_FATAL_STREAM("Unrecognized training method: " << training_method);
    return 1;
  }
  
  cout << "Training method: " << training_method << endl;
  cout << "Initialization sets: " << endl << init_paths;
  cout << "Training sets: " << endl << training_paths;
  cout << "Testing sets: " << endl << testing_paths;
  cout << "Output dir:  " << output_path;
  cout << endl;

  // -- Create output directory.
  if(bfs::exists(output_path)) {
    cout << "Output path " << output_path << " already exists.  Aborting." << endl;
    return 0;
  }
  bfs::create_directory(output_path);
  
  // -- Run initialization.
  cout << "Loading initialization dataset..." << endl;
  TrackDataset::Ptr init_data = loadDatasets(init_paths);
  GridClassifier::Ptr classifier(new GridClassifier);
  cout << "Initializing classifier..." << endl;
  classifier->initialize(*init_data, NUM_CELLS);
  init_data.reset();
  GridClassifier::StochasticLogisticTrainer trainer(classifier);
  
  // -- Run training.
  int idx = 0;
  cout << "Initializing buffer..." << endl;
  TrainingBuffer buffer;
  buffer.applyNameMappings(*classifier);
  buffer.init(1000);
  cout << "Training classifier..." << endl;
  int total = 0;
  for(int pass = 0; pass < NUM_PASSES; ++pass) { 
    for(size_t i = 0; i < training_paths.size(); ++i, ++idx) {
      cout << " ---- " << training_paths[i] << endl;
      cout << "Loading " << endl;
      TrackDataset::Ptr data(new TrackDataset);
      data->load(training_paths[i]);
      ROS_ASSERT(classifier->nameMappingsAreEqual(*data));
      ROS_ASSERT(data->nameMapping("cmap") == g_common_class_map);
      ROS_ASSERT(data->nameMapping("dmap") == g_common_descriptor_map);
      total += data->size();

      cout << "Merging..." << endl;
      buffer.merge(*data);
      cout << "Training buffer: " << endl;
      cout << buffer.status("  ") << endl;
      cout << "Training..." << endl;

      // Look at both the buffer and the new data, randomly mixing between them.
      vector<TrackDataset::ConstPtr> ds;
      ds.push_back(buffer.getTrackDataset());
      ds.push_back(data);
      trainer.train(ds);
      
      if(INCREMENTAL_EVAL) {
	ostringstream oss;
	oss << output_path + "/incremental_results";
	if(i == 0)
	  bfs::create_directory(oss.str());
	oss << "/eval" << setfill('0') << setw(3) << idx;
	
	evaluate(testing_paths, buffer.totalFrames(), classifier, oss.str());
      }
    }
  }

  cout << "Trained on a total of " << buffer.totalFrames() << " instances." << endl;

  // -- Evaluate.
  evaluate(testing_paths, buffer.totalFrames(), classifier, output_path + "/final_results");
  
  return 0;
}
