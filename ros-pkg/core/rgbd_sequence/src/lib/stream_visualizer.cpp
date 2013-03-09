#include <rgbd_sequence/stream_visualizer.h>

using namespace std;
using namespace pcl::visualization;

namespace rgbd
{

  StreamVisualizer::StreamVisualizer(StreamSequenceBase::ConstPtr sseq) :
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
      lockWrite();
      if(needs_update_) {
        if(pcd_->empty()) {
          Point pt;
          pt.x = 0;
          pt.y = 0;
          pt.z = 0;
          pcd_->push_back(pt);
        }
        if(!vis_.updatePointCloud(pcd_, "default"))
          vis_.addPointCloud(pcd_, "default");
        needs_update_ = false;
      }
      unlockWrite();
      
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
      handleKeypress(key);
    }
  }

  void StreamVisualizer::handleKeypress(char key)
  {
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
    cout <<  frame.img_.rows << " x " << frame.img_.cols << endl;
    cv::imshow ("FRAME", frame.img_);
    cout << "On frame " << idx_ << endl;
    cv::waitKey (100);
    needs_update_ = true;
  }

}  // namespace rgbd
