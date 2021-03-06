#include <online_learning/cross_evaluator.h>
#include <online_learning/evaluator.h>
#include <online_learning/grid_classifier.h>
#include <boost/filesystem.hpp>

using namespace std;
namespace bfs = boost::filesystem;

void CrossEvaluator::addTrackDataset(TrackDataset::ConstPtr td, std::string name)
{
  tds_.push_back(td);
  names_.push_back(name);
}

void CrossEvaluator::evaluateTrainingSetSize(std::string dir, size_t num_orderings, const std::vector<size_t>& num_cells, double thresh) const
{
  if(!bfs::exists(dir))
    bfs::create_directory(dir);
  
  // -- For each dataset, train on all the others and evaluate.
  GridClassifier::Ptr placeholder(new GridClassifier);
  placeholder->applyNameMappings(*tds_[0]);
  Evaluator ev_overall(placeholder);
  ev_overall.plot_ = false;
  for(size_t i = 0; i < tds_.size(); ++i) {
    // -- Set up test set.
    TrackDataset::ConstPtr test = tds_[i];
    ostringstream oss;
    oss << dir << "/" << names_[i];
    string dataset_dir = oss.str();
    bfs::create_directory(dataset_dir);
    ofstream file((dataset_dir + "/test.txt").c_str());
    file << test->status() << endl;
    file.close();

    // -- Make the remainder be the possible training sets.
    vector<TrackDataset::ConstPtr> all_training_sets;
    all_training_sets.reserve(tds_.size() - 1);
    for(size_t j = 0; j < tds_.size(); ++j)
      if(j != i)
        all_training_sets.push_back(tds_[j]);

    vector<size_t> indices(all_training_sets.size());
    for(size_t i = 0; i < indices.size(); ++i)
      indices[i] = i;
    
    // -- For many random orderings, run the evaluation.
    for(size_t j = 0; j < num_orderings; ++j) {
      ostringstream oss;
      oss << dataset_dir << "/ordering" << setw(4) << setfill('0') << j;
      string ordering_dir = oss.str();
      bfs::create_directory(ordering_dir);

      // We need to get the same sequence of orderings so that we can compare
      // between CrossEvaluator runs that use different descriptor pipelines.
      srand(j);
      
      random_shuffle(indices.begin(), indices.end());
      TrackDataset::Ptr train(new TrackDataset);
      train->applyNameMappings(*all_training_sets[0]);

      ofstream file((ordering_dir + "/ordering.txt").c_str());
      for(size_t k = 0; k < indices.size(); ++k)
        file << names_[indices[k]] << endl;
      file.close();

      for(size_t k = 0; k < all_training_sets.size(); ++k) {
        size_t idx = indices[k];
        *train += *all_training_sets[idx];

        ostringstream oss;
        oss << ordering_dir << "/" << setw(4) << setfill('0') << k;
        string subset_dir = oss.str();
        bfs::create_directory(subset_dir);
        ofstream file((subset_dir + "/train.txt").c_str());
        file << train->status() << endl;
        file.close();
        vector<Indices> indices;
        indices.push_back(Indices::All(train->size()));

        // -- Initialize GridClassifier
        GridClassifier::Ptr gc(new GridClassifier);
        gc->initialize(*train, num_cells);
    
        // -- Set up trainer.
        GridClassifier::BoostingTrainer trainer(gc);
        trainer.obj_thresh_ = thresh;
        trainer.gamma_ = 0;
        trainer.verbose_ = true;
        
        // -- Run the training.
        vector<TrackDataset::ConstPtr> foo;
        foo.push_back(train);
        trainer.train(foo, indices);

        // -- Run the evaluation.
        cout << "Evaluating" << endl;
        Evaluator ev(gc);
        ev.evaluateParallel(*test);
        ev.plot_ = false;
        ev.saveResults(subset_dir);

        if(j == 0 && k == all_training_sets.size() - 1) {
          ev_overall.classifier_ = gc;
          ev_overall.evaluateParallel(*test);
        }
      }
    }
  }

  ostringstream oss;
  oss << dir << "/overall";
  string overall_dir = oss.str();
  bfs::create_directory(overall_dir);
  ev_overall.saveResults(overall_dir);
}

void CrossEvaluator::evaluate(std::string dir, const std::vector<size_t>& num_cells, double thresh) const
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
    trainer.obj_thresh_ = thresh;
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
