#include <jarvis/blob_view.h>
#include <jarvis/tracker.h>
#define VTK_EXCLUDE_STRSTREAM_HEADERS
#include <pcl/visualization/pcl_visualizer.h>

BlobView::BlobView() :
  visualizer_(NULL),
  needs_update_(false),
  vis_(new Cloud)
{
}

BlobView::~BlobView()
{
  if(visualizer_)
    delete visualizer_;
}

void BlobView::_run()
{
  visualizer_ = new PCLVisualizer("Track View");
  
  visualizer_->registerKeyboardCallback(&BlobView::keyboardCallback, *this);
  visualizer_->addCoordinateSystem(0.2);
  visualizer_->setBackgroundColor(0.5, 0.5, 0.5);
  visualizer_->addPointCloud(Cloud::Ptr(new Cloud), "default");
  visualizer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "default");

  while(true) {

    lockWrite();
    if(needs_update_) {
      visualizer_->updatePointCloud(vis_, "default");
      visualizer_->removeShape("text");
      visualizer_->addText(message_, 10, 50, 16, 0.9, 0.9, 0.9, "text");
      needs_update_ = false;
    }
    unlockWrite();
    
    visualizer_->spinOnce(20);
  }

  delete visualizer_;
}

bool BlobView::keypress(pcl::visualization::KeyboardEvent* event, __attribute__((unused)) void* caller)
{
  scopeLockWrite;

  if(events_.empty())
    return false;

  *event = events_.back();
  events_.pop_back();
  return true;
}

void BlobView::keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* cookie)
{
  if(!event.keyDown())
    return;

  scopeLockWrite;
  events_.clear();  // Turns out having a buffer is really annoying.
  events_.push_back(event);
}

void BlobView::displayInstance(Instance& instance, __attribute__((unused)) void* caller)
{
  Blob::Ptr blob = boost::any_cast<Blob::Ptr>(instance.raw());
  if(!blob->cloud_)
    blob->project();
  
  scopeLockWrite;
  vis_ = blob->cloud_;
  needs_update_ = true;

  blob->clearProjected();
}

void BlobView::clearInstance(__attribute__((unused)) void* caller)
{
  scopeLockWrite;
  vis_ = Cloud::Ptr(new Cloud);
  needs_update_ = true;
}

void BlobView::displayMessage(const std::string& message, __attribute__((unused)) void* caller)
{
  scopeLockWrite;
  message_ = message;
  needs_update_ = true;
}
