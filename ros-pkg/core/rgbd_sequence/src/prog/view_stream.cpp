#include <boost/program_options.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <opencv2/highgui/highgui.hpp>
#include <rgbd_sequence/stream_sequence.h>
#include <bag_of_tricks/agent.h>

using namespace std;
using namespace rgbd;
using namespace pcl::visualization;
namespace bpo = boost::program_options;

class StreamVisualizer : public Agent
{
public:
  StreamVisualizer(StreamSequence::ConstPtr sseq);
  void _run();

protected:
  PCLVisualizer vis_;
  StreamSequence::ConstPtr sseq_;
  int idx_;
  Cloud::Ptr pcd_;
  
  void pointPickingCallback(const pcl::visualization::PointPickingEvent& event, void* cookie);
  void keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* cookie);
  void increment(int num);
};

StreamVisualizer::StreamVisualizer(StreamSequence::ConstPtr sseq) :
  sseq_(sseq),
  idx_(0),
  pcd_(new Cloud)
{
  vis_.addCoordinateSystem(0.25);
  vis_.setBackgroundColor(255, 255, 255);
  vis_.registerPointPickingCallback(&StreamVisualizer::pointPickingCallback, *this);
  vis_.registerKeyboardCallback(&StreamVisualizer::keyboardCallback, *this);

  increment(0);
}

void StreamVisualizer::_run()
{
  while(true) {
    if(pcd_->empty()) {
      Point pt;
      pt.x = 0;
      pt.y = 0;
      pt.z = 0;
      pcd_->push_back(pt);
    }
    if(!vis_.updatePointCloud(pcd_, "default"))
      vis_.addPointCloud(pcd_, "default");

    vis_.spinOnce();

    scopeLockWrite;
    if(quitting_)
      break;
  }
}

void StreamVisualizer::keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* cookie)
{
  if(event.keyDown()) {
    char key = event.getKeyCode();
    if(key == 27) {
      scopeLockWrite;
      quitting_ = true;
    }
    else if(key == '>')
      increment(10);
    else if(key == '<')
      increment(-10);
    else if(key == '.')
      increment(1);
    else if(key == ',')
      increment(-1);
  }
}

void StreamVisualizer::pointPickingCallback(const pcl::visualization::PointPickingEvent& event, void* cookie)
{
  if(event.getPointIndex() == -1)
    return;
    
  Point pt;
  event.getPoint(pt.x, pt.y, pt.z);
  cout << "Selected point: " << pt.x << ", " << pt.y << ", " << pt.z << endl;
  vis_.removeAllShapes();
  
  Point origin;
  origin.x = 0;
  origin.y = 0;
  origin.z = 0;
  vis_.addArrow<Point, Point>(origin, pt, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, "line");
}

void StreamVisualizer::increment(int num)
{
  scopeLockWrite;

  vis_.removeAllShapes();
  
  idx_ += num;
  idx_ = max(0, idx_);
  idx_ = min((int)sseq_->size(), idx_);

  Frame frame;
  sseq_->readFrame(idx_, &frame);
  sseq_->model_.frameToCloud(frame, pcd_.get());
}


int main(int argc, char** argv)
{
  string dir;
  bpo::options_description opts_desc("Allowed options");
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("distortion-model", bpo::value<string>(), "Use pre-computed distortion model")
    ("seq", bpo::value<string>(&dir), "Sequence directory")
    ("only-stats", "Only print stats and exit")
    ;

  bpo::positional_options_description p;
  p.add("seq", 1);
  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  if(opts.count("help")) {
    cout << "Usage: view_stream SEQ [opts]" << endl << endl;
    cout << opts_desc << endl;
    return 1;
  }
  bpo::notify(opts);
  
  cout << "Looking at dir: " << dir << endl;
  StreamSequence::Ptr sseq(new StreamSequence);
  sseq->load(dir);
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


  StreamVisualizer vis(sseq);
  vis.run();
  return 0;

  
  // pcl::visualization::PCLVisualizer vis("cloud");
  // //pcl::visualization::CloudViewer cloud_viewer("cloud");
  // //vis.registerPointPickingCallback(pointPickingCallback);
  // vis.registerKeyboardCallback(keyboardCallback);
  // cv::namedWindow("image");
  // Frame frame;
  // size_t idx = 0;
  // while(idx < seq.size()) {


  //   cout << "Viewing cloud: " << idx << endl;
  //   seq.readFrame(idx, &frame);

  // }
  // for(size_t i = 0; i < seq.size(); i++){
  //   cout << "Viewing cloud: " << i << endl;
  //   seq.readFrame(i, &frame);

  //   Cloud::Ptr cloud(new Cloud);
  //   seq.model_.frameToCloud(frame, cloud.get());
  //   vis.showCloud(cloud);

  //   cv::imshow("image", frame.img_);

  //   if(i < seq.size()) {
  //     double dt = seq.timestamps_[i+1]-seq.timestamps_[i];
  //     cout << "dt: " << dt << endl;
  //     cv::waitKey(dt * 1e3);
  //   }
  // }
  
  return 0;
}


