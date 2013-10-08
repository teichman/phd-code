#include <online_learning/tbssl.h>
#include <boost/program_options.hpp>
#include <boost/foreach.hpp>

using namespace std;
using namespace Eigen;
namespace bpo = boost::program_options;
namespace bfs = boost::filesystem;

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  vector<string> dataset_paths;
  string output_dir;
  vector<size_t> num_cells;
  num_cells.push_back(10);
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("tds", bpo::value(&dataset_paths)->required()->multitoken(), "Labeled data")
    ("output,o", bpo::value(&output_dir)->required(), "Output directory")
    ("num-cells", bpo::value< vector<size_t> >(&num_cells)->multitoken(), "Default is --num-cells 10.")
    ;

  p.add("tds", -1);  // -1 means unlimited

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

  // -- Get dataset names.
  vector<string> dataset_names(dataset_paths.size());
  for(size_t i = 0; i < dataset_paths.size(); ++i) {
    bfs::path path(dataset_paths[i]);
    dataset_names[i] = path.stem().string();
    cout << "Name: " << dataset_names[i] << endl;
  }
      
  // -- Load all datasets.
  vector<TrackDataset::ConstPtr> tds;
  for(size_t i = 0; i < dataset_paths.size(); ++i) {
    cout << "Loading " << dataset_paths[i] << endl;
    TrackDataset::Ptr td(new TrackDataset);
    td->load(dataset_paths[i]);
    if(i > 0)
      ROS_ASSERT(td->nameMappingsAreEqual(*tds[i-1]));
    tds.push_back(td);
  }

  if(!bfs::exists(output_dir))
    bfs::create_directory(output_dir);
  
  // -- For each dataset, train on all the others and evaluate.
  GridClassifier::Ptr placeholder(new GridClassifier);
  placeholder->applyNameMappings(*tds[0]);
  Evaluator ev_overall(placeholder);
  ev_overall.plot_ = false;
  for(size_t i = 0; i < tds.size(); ++i) {
    // -- Set up training and test sets.
    TrackDataset::ConstPtr test_dataset = tds[i];
    TrackDataset::Ptr training_dataset(new TrackDataset);
    training_dataset->applyNameMappings(*tds[0]);
    for(size_t j = 0; j < tds.size(); ++j)
      if(j != i)
        *training_dataset += *tds[j];
    vector<Indices> indices;
    indices.push_back(Indices::All(training_dataset->size()));

    // -- Initialize GridClassifier
    GridClassifier::Ptr gc(new GridClassifier);
    gc->initialize(*training_dataset, num_cells);
    
    // -- Set up trainer.
    GridClassifier::BoostingTrainer trainer(gc);
    trainer.gamma_ = 0;
    trainer.verbose_ = true;

    // -- Run the training.
    vector<TrackDataset::ConstPtr> training_datasets;
    training_datasets.push_back(training_dataset);
    trainer.train(training_datasets, indices);

    // -- Run the evaluation.
    cout << "Evaluating" << endl;
    Evaluator ev(gc);
    ev.evaluateParallel(*test_dataset);
    ev.plot_ = false;
    ostringstream oss;
    oss << output_dir << "/" << dataset_names[i];
    bfs::create_directory(oss.str());
    ev.saveResults(oss.str());

    ev_overall.classifier_ = gc;
    ev_overall.evaluateParallel(*test_dataset);
  }

  ev_overall.saveResults(output_dir);
  return 0;
}
