#include <online_learning/cross_evaluator.h>

using namespace std;
namespace bfs = boost::filesystem;

void CrossEvaluator::addTrackDataset(TrackDataset::ConstPtr td, std::string name)
{
  tds_.push_back(td);
  names_.push_back(name);
}

void CrossEvaluator::evaluate(std::string dir, const std::vector<size_t>& num_cells) const
{
  if(!bfs::exists(dir))
    bfs::create_directory(dir);
  
  // -- For each dataset, train on all the others and evaluate.
  GridClassifier::Ptr placeholder(new GridClassifier);
  placeholder->applyNameMappings(*tds_[0]);
  Evaluator ev_overall(placeholder);
  ev_overall.plot_ = false;
  for(size_t i = 0; i < tds_.size(); ++i) {
    // -- Set up training and test sets.
    TrackDataset::ConstPtr test_dataset = tds_[i];
    TrackDataset::Ptr training_dataset(new TrackDataset);
    training_dataset->applyNameMappings(*tds_[0]);
    for(size_t j = 0; j < tds_.size(); ++j)
      if(j != i)
        *training_dataset += *tds_[j];
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
    oss << dir << "/" << names_[i];
    bfs::create_directory(oss.str());
    ev.saveResults(oss.str());

    ev_overall.classifier_ = gc;
    ev_overall.evaluateParallel(*test_dataset);
  }

  ev_overall.saveResults(dir);
}
