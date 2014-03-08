#include <online_learning/track_dataset_visualizer.h>

class BlobView : public TrackView, public Agent
{
public:
  BlobView();
  virtual ~BlobView();
  void displayInstance(Instance& instance, void* caller);
  void clearInstance(void* caller);
  void displayMessage(const std::string& message, void* caller);
  bool keypress(pcl::visualization::KeyboardEvent* event, void* caller);
  
  void _run();
  
protected:
  typedef pcl::visualization::PCLVisualizer PCLVisualizer;
  PCLVisualizer* visualizer_;
  bool needs_update_;
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > vis_;
  std::string message_;
  std::vector<pcl::visualization::KeyboardEvent> events_;

  void keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* cookie);
};
