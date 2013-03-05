#include <xpl_calibration/link_visualizer.h>

using namespace std;
using namespace Eigen;
using namespace rgbd;

LinkVisualizer::LinkVisualizer(StreamSequenceBase::ConstPtr sseq, PoseGraphSlam::Ptr slam) :
  sseq_(sseq),
  slam_(slam),
  quitting_(false),
  needs_update_(false),
  transform_(true),
  edge_idx_(0),
  sorted_(false),
  frame_text_(""),
  view_mode_(ALL)
{
  vis_.registerKeyboardCallback(&LinkVisualizer::keyboardCallback, *this);
  vis_.setBackgroundColor(1,1,1);
  // -- Set the viewpoint to be sensible for PrimeSense devices.
  // SDM TODO vis_.camera_.clip[0] = 0.00387244;
  // SDM TODO vis_.camera_.clip[1] = 3.87244;
  // SDM TODO vis_.camera_.focal[0] = -0.160878;
  // SDM TODO vis_.camera_.focal[1] = -0.0444743;
  // SDM TODO vis_.camera_.focal[2] = 1.281;
  // SDM TODO vis_.camera_.pos[0] = 0.0402195;
  // SDM TODO vis_.camera_.pos[1] = 0.0111186;
  // SDM TODO vis_.camera_.pos[2] = -1.7;
  // SDM TODO vis_.camera_.view[0] = 0;
  // SDM TODO vis_.camera_.view[1] = -1;
  // SDM TODO vis_.camera_.view[2] = 0;
  // SDM TODO vis_.camera_.window_size[0] = 1678;
  // SDM TODO vis_.camera_.window_size[1] = 525;
  // SDM TODO vis_.camera_.window_pos[0] = 2;
  // SDM TODO vis_.camera_.window_pos[1] = 82;
  // SDM TODO vis_.updateCamera();    
}

void LinkVisualizer::run()
{
  // Solve slam so we can sort
  slam_->solve(); 
  errors_.resize(slam_->edges_.size());
  for(size_t i = 0; i < slam_->edges_.size(); i++)
  {
    const EdgeStruct &e = slam_->edges_[i];
    const Eigen::Affine3d &pairwise_trans = e.transform;
    Eigen::Affine3d predicted_pairwise_trans = slam_->transform(e.idx0).inverse()*slam_->transform(e.idx1);
    errors_[i] = (pairwise_trans.matrix() - predicted_pairwise_trans.matrix()).norm();
  }
  sortv(errors_, errors_desc_, edges_desc_, DESCENDING);
  incrementEdgeIdx(0);
  visualizationThreadFunction();
}

void LinkVisualizer::setCamera(const std::string& camera_path)
{
  int argc = 3;
  char* argv[argc];
  argv[0] = (char*)string("aoeu").c_str();
  argv[1] = (char*)string("-cam").c_str();
  argv[2] = (char*)camera_path.c_str();
  bool success = vis_.getCameraParameters(argc, argv);
  ROS_ASSERT(success);
  vis_.updateCamera();    
}

void LinkVisualizer::visualizationThreadFunction()
{
  while(true) {
    { scopeLockRead; if(quitting_) break; }
    usleep(5e3);
    
    lockWrite();
    if(needs_update_) {
      Cloud::Ptr prev = sseq_->getCloud(cur_edge_->idx0);
      if(view_mode_ == PREV || view_mode_ == ALL)
      {
        if(!vis_.updatePointCloud(prev, "prev"))
    vis_.addPointCloud(prev, "prev");
      } else
      {
        vis_.removePointCloud("prev");
      }
      Cloud::Ptr curr = sseq_->getCloud(cur_edge_->idx1);
      Cloud::Ptr curr_trans;
      if(transform_)
      {
        curr_trans = Cloud::Ptr(new Cloud);
        pcl::transformPointCloud(*curr, *curr_trans, cur_edge_->transform.cast<float>());
      } else
      {
        curr_trans = curr;
      }
      if(view_mode_ == CUR || view_mode_ == ALL)
      {
        if(!vis_.updatePointCloud(curr_trans, "curr_trans"))
    vis_.addPointCloud(curr_trans, "curr_trans");
      } else
      {
        vis_.removePointCloud("curr_trans");
      }
      int xpos = 20; int ypos = 10; int fontsize = 25;
      string toshow = frame_text_;
      if(transform_) toshow += " (after)";
      else toshow += " (before)";
      if(!vis_.updateText(toshow, xpos, ypos, fontsize, 0, 0, 0, "label"))
  vis_.addText(toshow, xpos, ypos, fontsize, 0, 0, 0, "label");
      needs_update_ = false;
    }
    unlockWrite();
    vis_.spinOnce(3);
  }
}

void LinkVisualizer::keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* cookie)
{
  if(event.keyDown()) {
    char key = event.getKeyCode();
    if(key == 27) {
      scopeLockWrite;
      quitting_ = true;
    }
    else if(key == '.')
      incrementEdgeIdx(1);
    else if(key == ',')
      incrementEdgeIdx(-1);
    else if(key == '>')
      incrementEdgeIdx(10);
    else if(key == '<')
      incrementEdgeIdx(-10);
    else if(key == ' ')
      toggleTransform();
    else if(key == 's')
    {
      toggleSort();
      incrementEdgeIdx(0);
    }
    else if(key == 'z')
      setViewMode(PREV);
    else if(key == 'x')
      setViewMode(CUR);
    else if(key == 'c')
      setViewMode(ALL);
    else
      cout << "key: " << (int)key << endl;
  }
}

void LinkVisualizer::incrementEdgeIdx(int num)
{
  int idx = (int) edge_idx_;
  scopeLockWrite;
  int edge_idx = (idx + num) % (int) slam_->edges_.size();
  if(edge_idx < 0) edge_idx += slam_->edges_.size();
  cout << "Edge_idx = " << edge_idx << " / " << slam_->edges_.size() << endl;
  edge_idx_ = edge_idx;
  cout << "Edge_idx_ = " << edge_idx_ << " / " << slam_->edges_.size() << endl;
  cur_edge_ = sorted_ ? &slam_->edges_[edges_desc_[edge_idx_]] : &slam_->edges_[edge_idx_];
  transform_ = true;
  ostringstream oss;
  oss << "Edge " << cur_edge_->idx1 << "->" << cur_edge_->idx0;
  oss << " [error: " << (sorted_ ? errors_desc_[edge_idx_] : errors_[edge_idx_]) << "]" << endl;
  frame_text_ = oss.str();
  view_mode_ = ALL;
  needs_update_ = true;
}

void LinkVisualizer::toggleTransform()
{
  scopeLockWrite;
  transform_ = !transform_;
  needs_update_ = true;
}

void LinkVisualizer::toggleSort()
{
  scopeLockWrite;
  sorted_ = !sorted_;
  edge_idx_ = 0;
  cout << "Sorted: " << sorted_ << endl;
  needs_update_ = true;
}

void LinkVisualizer::setViewMode(ViewMode mode)
{
  cout << "View Mode: " << mode << endl;
  scopeLockWrite;
  view_mode_ = mode;
  needs_update_ = true;

}
