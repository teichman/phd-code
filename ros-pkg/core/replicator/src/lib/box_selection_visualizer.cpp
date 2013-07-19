#include <replicator/box_selection_visualizer.h>

using namespace std;
using namespace pcl;
using namespace pcl::visualization;
using namespace rgbd;

BoxSelectionVisualizer::BoxSelectionVisualizer(Cloud::Ptr pcd, std::string output_path) :
  pcd_(pcd),
  pcdvis_(new Cloud),
  center_(0, 0, 1.5),
  width_x_(0.5),
  width_y_(0.5),
  width_z_(0.5),
  needs_update_(true),
  output_path_(output_path)
{
  vis_.registerKeyboardCallback(&BoxSelectionVisualizer::keyboardCallback, *this);
  vis_.registerPointPickingCallback(&BoxSelectionVisualizer::pointPickingCallback, *this);
  vis_.setBackgroundColor(1, 1, 1);
  vis_.addCoordinateSystem(0.3, 0, 0, 0);
}

void BoxSelectionVisualizer::keyboardCallback(const KeyboardEvent& event,
                                              void* cookie)
{
  if(event.keyDown()) {
    char key = event.getKeyCode();
    scopeLockWrite;
    if(key == 27) {
      quitting_ = true;
    }
    else if(key == 'x') {
      width_x_ += 0.1;
      needs_update_ = true;
      cout << "Dimensions: " << width_x_ << " x " << width_y_ << " x " << width_z_ << endl;
    }
    else if(key == 'X') {
      width_x_ -= 0.1;
      needs_update_ = true;
      cout << "Dimensions: " << width_x_ << " x " << width_y_ << " x " << width_z_ << endl;
    }
    else if(key == 'y') {
      width_y_ += 0.1;
      needs_update_ = true;
      cout << "Dimensions: " << width_x_ << " x " << width_y_ << " x " << width_z_ << endl;
    }
    else if(key == 'Y') {
      width_y_ -= 0.1;
      needs_update_ = true;
      cout << "Dimensions: " << width_x_ << " x " << width_y_ << " x " << width_z_ << endl;
    }
    else if(key == 'z') {
      width_z_ += 0.1;
      needs_update_ = true;
      cout << "Dimensions: " << width_x_ << " x " << width_y_ << " x " << width_z_ << endl;
    }
    else if(key == 'Z') {
      width_z_ -= 0.1;
      needs_update_ = true;
      cout << "Dimensions: " << width_x_ << " x " << width_y_ << " x " << width_z_ << endl;
    }
    else if(key == 's')
      save();
  }
}

void BoxSelectionVisualizer::save()
{
  Cloud selected;
  selected.reserve(pcd_->size());
  for(size_t i = 0; i < inside_.size(); ++i)
    if(inside_[i])
      selected.push_back(pcd_->at(i));

  pcl::io::savePCDFileBinary(output_path_, selected);
  pcl::io::savePCDFileASCII(output_path_ + ".txt", selected);
  cout << "Saved to " << output_path_ << endl;
  cout << "Saved to " << output_path_ + ".txt" << endl;
}

void BoxSelectionVisualizer::pointPickingCallback(const PointPickingEvent& event,
                                                  void* cookie)
{
  cout << "pointpickingevent" << endl;
  if(event.getPointIndex() == -1)
    return;
  
  Point pt;
  event.getPoint(pt.x, pt.y, pt.z);
  cout << "Selected point: " << pt.x << ", " << pt.y << ", " << pt.z << endl;
  center_ = pt;
  
  needs_update_ = true;
}

void BoxSelectionVisualizer::_run()
{
  while(true) {
    { scopeLockRead; if(quitting_) break; }
    if(needs_update_) {
      drawVisualization();
      if(!vis_.updatePointCloud(pcdvis_, "default"))
        vis_.addPointCloud(pcdvis_, "default");
      needs_update_ = false;
    }
    vis_.spinOnce(3);
  }
}

void BoxSelectionVisualizer::drawVisualization()
{
  scopeLockWrite;
  
  // -- Determine who's inside.
  inside_.clear();
  inside_.resize(pcd_->size(), false);
  for(size_t i = 0; i < pcd_->size(); ++i) {
    const Point& pt = pcd_->at(i);
    if(pt.x < center_.x + width_x_ / 2 && pt.x > center_.x - width_x_ / 2 &&
       pt.y < center_.y + width_y_ / 2 && pt.y > center_.y - width_y_ / 2 &&
       pt.z < center_.z + width_z_ / 2 && pt.z > center_.z - width_z_ / 2)
    {
      inside_[i] = true;
    }
  }

  // -- Color the points.
  pcdvis_->clear();
  pcdvis_->reserve(pcd_->size());
  for(size_t i = 0; i < inside_.size(); ++i) {
    Point pt = pcd_->at(i);
    if(inside_[i])
      pcdvis_->push_back(pt);
    else {
      pt.r = 127;
      pt.g = 127;
      pt.b = 127;
      pcdvis_->push_back(pt);
    }
  }
}

