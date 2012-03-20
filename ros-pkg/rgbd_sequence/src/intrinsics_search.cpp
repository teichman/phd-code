#include <rgbd_sequence/stream_sequence.h>
#include <rgbd_sequence/projector.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/impl/extract_clusters.hpp>

using namespace std;
using namespace Eigen;
using namespace pcl;
using namespace pcl::visualization;
using namespace rgbd;

string usageString()
{
  ostringstream oss;
  oss << "Usage: intrinsics_search SEQ" << endl;
  oss << "  SEQ is a sequence made for this purpose." << endl;
  return oss.str();
}

class VisWrapper
{
public:
  VisWrapper() :
    key_(0)
  {
    vis_.registerKeyboardCallback(&VisWrapper::keyboardCallback, *this);
    Cloud::Ptr pcd(new Cloud);
    vis_.addPointCloud(pcd, "default");
  }
  
  char waitKey()
  {
    if(vis_.wasStopped())
      return -1;
    
    key_ = 0;
    while(key_ == 0)
      usleep(1e3);
    return key_;
  }
  
  void keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* cookie)
  {
    if(event.keyDown())
      key_ = event.getKeyCode();
  }

  void showCloud(rgbd::Cloud::ConstPtr pcd)
  {
    vis_.updatePointCloud(pcd, "default");
  }
  
protected:
  char key_;
  pcl::visualization::PCLVisualizer vis_;
};

int main(int argc, char** argv)
{
  if(argc != 2) {
    cout << usageString() << endl;
    return -1;
  }
  
  StreamSequence sseq;
  sseq.load(argv[1]);

  Cloud::Ptr pcd = sseq.getCloud(1);
  VisWrapper vis;
  vis.showCloud(pcd);

  while(true) {
    char key = vis.waitKey();
    switch(key) {
    case -1:
    case 'q':
      return 0;
      break;
    default:
      cout << "Pressed " << (int)key << endl;
      break;
    }
  }
  
  return 0;
}
