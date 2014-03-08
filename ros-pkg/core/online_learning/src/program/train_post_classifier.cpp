#include <online_learning/tbssl.h>
#include <boost/program_options.hpp>

using namespace std;
namespace bpo = boost::program_options;

int main(int argc, char** argv) {
   
// -- Parse args.
  string online_learner_path;
  string output_path;

  bpo::positional_options_description p;
  bpo::options_description opts_desc("Allowed options");
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("ol", bpo::value(&online_learner_path), ".ol file")
    ("output,o", bpo::value<string>(&output_path), "location to save classifier")
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

  cout << "Loading online learner at " << online_learner_path << endl;
  OnlineLearner ol((IfstreamWrapper(online_learner_path)));

  TrackDataset::Ptr train(new TrackDataset);
  train->applyNameMappings(ol);
  *train += *ol.annotated();
  *train += *ol.autobg();
  *train += *ol.unsupervised();

  cout << "Initializing." << endl;
  GridClassifier::Ptr classifier(new GridClassifier);
  vector<size_t> num_cells;
  num_cells.push_back(10);
  classifier->initialize(*train, num_cells);
  classifier->applyNameMappings(ol);

  vector<TrackDataset::ConstPtr> datasets;
  datasets.push_back(train);
  vector<Indices> indices;
  for(size_t i = 0; i < datasets.size(); ++i)
    indices.push_back(Indices::All(datasets[i]->size()));
  
  GridClassifier::BoostingTrainer trainer(classifier);
  trainer.verbose_ = true;
  trainer.train(datasets, indices);

  cout << "Saving to " << output_path << "." << endl;
  classifier->save(output_path);

  return 0;
}
