#include <jarvis/induction_supervisor.h>

using namespace std;
using namespace Eigen;

InductionSupervisor::InductionSupervisor(GridClassifier gc, YAML::Node config,
                                         const Eigen::VectorXf& up, OnlineLearner* ol,
                                         float conf_thresh, int period, std::string output_dir) :
  annotation_limit_(-1),
  max_iter_to_supervise_(-1),
  gc_(gc),
  config_(config),
  up_(up),
  ol_(ol),
  conf_thresh_(conf_thresh),
  period_(period),
  output_dir_(output_dir)
{
}

void InductionSupervisor::_run()
{
  int iter = 0;
  int last_iter_provided = -1;
  
  while(!quitting_) {
    usleep(1e6);
    if(ol_->iter() < period_ || (ol_->iter() % period_) != 0)
      continue;
    // If we've already provided supervision for this iteration, don't do it again.
    if(ol_->iter() == last_iter_provided)
      continue;
    if(max_iter_to_supervise_ != -1 && ol_->iter() > max_iter_to_supervise_)
      continue;
    
    last_iter_provided = ol_->iter();
    
    for(size_t i = 0; i < gc_.nameMapping("cmap").size(); ++i) {
      string cname = gc_.nameMapping("cmap").toName(i);

      // -- Get a sample of inducted tracks.
      TrackDataset td = ol_->requestInductedSample(cname, 100);
      cout << "[InductionSupervisor] Got " << td.size() << " tracks for class " << cname << endl;
      if(td.empty())
        continue;
      
      updateDescriptors(config_["Pipeline"], 24, &td, up_);
      if(!gc_.nameMappingsAreEqual(td)) {
        cout << gc_.nameMappingStatus() << endl;
        cout << "---" << endl;
        cout << td.nameMappingStatus() << endl;
      }
      ROS_ASSERT(gc_.nameMappingsAreEqual(td));
      
      // -- Classify tracks and save errors to a new dataset.
      TrackDataset::Ptr errors(new TrackDataset);
      errors->applyNameMappings(td);
      for(size_t j = 0; j < td.size(); ++j) {
        if(annotation_limit_ != -1 && (int)(ol_->numAnnotated() + errors->size()) >= annotation_limit_) {
          cout << "[InductionSupervisor] Annotation limit reached." << endl;
          break;
        }

        Label prediction = gc_.classifyTrack(td[j]);
        cout << "[InductionSupervisor] supervisor prediction: " << prediction(i)
             << ", GI prediction: " << td.label(j)(i) << endl;

        if(fabs(prediction(i)) < conf_thresh_)
          continue;
        if(prediction(i) * td.label(j)(i) >= 0)
          continue;
        
        cout << "[InductionSupervisor] adding to errors." << endl;
        errors->tracks_.push_back(td.tracks_[j]);
        Label corrected = VectorXf::Zero(prediction.rows());
        corrected(i) = prediction.sign()(i);
        errors->tracks_.back()->setLabel(corrected);
      }
      cout << "[InductionSupervisor] Sending " << errors->size() << " errors." << endl;
      
      // -- Send feedback to ol_.
      if(!errors->empty())
        ol_->pushHandLabeledDataset(errors);

      ++iter;
    }
  }
}
