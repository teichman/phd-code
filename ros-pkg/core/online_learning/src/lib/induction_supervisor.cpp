#include <online_learning/induction_supervisor.h>

using namespace std;
using namespace Eigen;

InductionSupervisor::InductionSupervisor(GridClassifier gc, OnlineLearner* ol,
                                         float conf_thresh, std::string output_dir) :
  gc_(gc),
  ol_(ol),
  conf_thresh_(conf_thresh),
  output_dir_(output_dir)
{
}

void InductionSupervisor::_run()
{
  int iter = 0;
  while(!quitting_) {
    usleep(1e7);
    for(size_t i = 0; i < gc_.nameMapping("cmap").size(); ++i) {
      string cname = gc_.nameMapping("cmap").toName(i);

      // -- Get a sample of inducted tracks.
      float val = 0;
      if(iter % 2)
        val = (5 * (double)rand() / RAND_MAX) - 2.5;

      TrackDataset td = ol_->requestInductedSample(cname, val, 50);
      cout << "[InductionSupervisor] Got " << td.size() << " tracks for class " << cname << endl;
      ol_->entryHook(&td);  // Compute descriptors.
      ROS_ASSERT(gc_.nameMappingsAreEqual(td));
      
      // -- Classify tracks and save errors to a new dataset.
      TrackDataset::Ptr errors(new TrackDataset);
      errors->applyNameMappings(td);
      for(size_t j = 0; j < td.size(); ++j) {
        Label prediction = gc_.classifyTrack(td[j]);
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
