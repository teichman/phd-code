#include <boost/program_options.hpp>
#include <rgbd_sequence/stream_visualizer.h>
#include <rgbd_sequence/stream_sequence_base.h>
#include <rgbd_sequence/discrete_depth_distortion_model.h>

using namespace std;
using namespace rgbd;
using namespace pcl::visualization;
namespace bpo = boost::program_options;

class UndistortingStreamVisualizer : public StreamVisualizer
{
public:
  //! UndistortingStreamVisualizer does not delete DiscreteDepthDistortionModel.
  DiscreteDepthDistortionModel* dddm_;
  UndistortingStreamVisualizer(StreamSequenceBase::ConstPtr sseq);

protected:
  bool use_model_;
  
  void handleKeypress(char key);
  void increment(int num);
};


UndistortingStreamVisualizer::UndistortingStreamVisualizer(StreamSequenceBase::ConstPtr sseq) :
  StreamVisualizer(sseq),
  dddm_(NULL),
  use_model_(false)
{
}

void UndistortingStreamVisualizer::handleKeypress(char key)
{
  StreamVisualizer::handleKeypress(key);

  if(key == 'd') {
    lockWrite();
    use_model_ = !use_model_;
    if(use_model_ && !dddm_) {
      cout << "Must provide distortion model." << endl;
      use_model_ = false;
    }
    cout << "use_model_: " << use_model_ << endl;
    unlockWrite();
    
    increment(0);
  }
}

void UndistortingStreamVisualizer::increment(int num)
{
  scopeLockWrite;
  
  vis_.removeAllShapes();
  
  idx_ += num;
  idx_ = max(0, idx_);
  idx_ = min((int)sseq_->size(), idx_);
  
  Frame frame;
  sseq_->readFrame(idx_, &frame);
  if(use_model_ && dddm_)
    dddm_->undistort(&frame);
  
  sseq_->model_.frameToCloud(frame, pcd_.get());

  needs_update_ = true;
}

int main(int argc, char** argv)
{
  string dir;
  bpo::options_description opts_desc("Allowed options");
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("seq", bpo::value<string>(&dir), "Sequence directory")
    ("intrinsics", bpo::value<string>(), "Discrete depth distortion model")
    ("only-stats", "Only print stats and exit")
    ;

  bpo::positional_options_description p;
  p.add("seq", 1);
  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  if(opts.count("help")) {
    cout << "Usage: view_stream [ OPTS ] SEQ" << endl << endl;
    cout << opts_desc << endl;
    return 1;
  }
  bpo::notify(opts);
  
  cout << "Looking at dir: " << dir << endl;
  StreamSequenceBase::Ptr sseq = StreamSequenceBase::initializeFromDirectory(dir);
  cout << "Loaded successfully" << endl;

  double mean_dt = 0;
  double max_dt = -std::numeric_limits<double>::max();
  for(size_t i = 1; i < sseq->size(); i++) {
    double dt = sseq->timestamps_[i] - sseq->timestamps_[i-1];
    mean_dt += dt;
    if(dt > max_dt)
      max_dt = dt;
  }
  mean_dt /= (double)sseq->size();
  cout << "--------------------" << endl;
  cout << "Mean fps: " << (double)sseq->size() / (sseq->timestamps_.back() - sseq->timestamps_.front()) << endl;
  cout << "Mean dt: " << mean_dt << endl;
  cout << "Max dt: " << max_dt << endl;
  cout << "--------------------" << endl;
  cout << "Sensor model: " << endl;
  cout << sseq->model_.status("  ");
  
  if(opts.count("only-stats"))
    return 0;

  DiscreteDepthDistortionModel dddm;
  UndistortingStreamVisualizer vis(sseq);
  if(opts.count("intrinsics")) {
    dddm.load(opts["intrinsics"].as<string>());
    vis.dddm_ = &dddm;
  }
  vis.run();
  
  return 0;
}


