#include <online_learning/induction_supervisor.h>

using namespace std;
using namespace Eigen;

InductionSupervisor::InductionSupervisor(OnlineLearner* ol, float conf_thresh, std::string output_dir) :
  ol_(ol),
  conf_thresh_(conf_thresh),
  output_dir_(output_dir)
{
}

void InductionSupervisor::train(TrackDataset::Ptr td,
                                const std::vector<size_t>& nc,
                                double obj_thresh)
{
  gc_ = GridClassifier::Ptr(new GridClassifier);
  gc_->initialize(*td, nc);
  
  GridClassifier::BoostingTrainer trainer(gc_);
  trainer.verbose_ = true;
  trainer.gamma_ = 0;
  trainer.obj_thresh_ = obj_thresh;
  trainer.applyNameMappings(*gc_);

  vector<TrackDataset::ConstPtr> datasets;
  datasets.push_back(td);
  vector<Indices> indices;
  for(size_t i = 0; i < datasets.size(); ++i)
    indices.push_back(Indices::All(datasets[i]->size()));
  trainer.train(datasets, indices);
}

void InductionSupervisor::_run()
{
  ROS_ASSERT(gc_);

  int iter = 0;
  while(!quitting_) {
    usleep(1e7);
    for(size_t i = 0; i < gc_->nameMapping("cmap").size(); ++i) {
      string cname = gc_->nameMapping("cmap").toName(i);

      // -- Get a sample of inducted tracks.
      float val = (5 * (double)rand() / RAND_MAX) - 2.5;
      TrackDataset td = ol_->requestInductedSample(cname, val, 20);
      cout << "[InductionSupervisor] Got " << td.size() << " tracks for class " << cname << endl;
      ol_->entryHook(&td);  // Compute descriptors.
      ROS_ASSERT(gc_->nameMappingsAreEqual(td));
      
      // -- Classify tracks and save errors to a new dataset.
      TrackDataset::Ptr errors(new TrackDataset);
      errors->applyNameMappings(td);
      for(size_t j = 0; j < td.size(); ++j) {
        Label prediction = gc_->classifyTrack(td[j]);
        cout << "[InductionSupervisor] supervisor prediction: " << prediction(i)
             << ", GI prediction: " << td.label(j)(i) << endl;
        
        if(fabs(prediction(i)) > conf_thresh_ && prediction(i) * td.label(j)(i) < 0) {
          cout << "[InductionSupervisor] adding to errors." << endl;
          errors->tracks_.push_back(td.tracks_[j]);
          Label corrected = VectorXf::Zero(prediction.rows());
          corrected(i) = prediction.sign()(i);
          errors->tracks_.back()->setLabel(corrected);
        }
      }
      cout << "[InductionSupervisor] Found " << errors->size() << " errors." << endl;
      
      // -- Send feedback to ol_.
      if(!errors->empty())
        ol_->pushHandLabeledDataset(errors);

      ostringstream oss;
      oss << output_dir_ << "/InductionSupervisor-" << setw(4) << setfill('0') << iter << "-received.td";
      td.save(oss.str());

      oss.str("");
      oss << output_dir_ << "/InductionSupervisor-" << setw(4) << setfill('0') << iter << "-corrections.td";
      errors->save(oss.str());
      
      ++iter;
    }
  }
}
