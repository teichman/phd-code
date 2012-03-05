#include <rgbd_sequence/compact_sequence.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;
using namespace rgbd;

string usageString()
{
  ostringstream oss;
  oss << "Usage: view_compact DIR" << endl;
  oss << "Where DIR is the directory of the compact sequence" << endl;
  return oss.str();
}

int main(int argc, char** argv)
{
  if(argc < 2) {
    cout << usageString() << endl;
    return 0;
  }
  string dir = argv[1];
  cout << "Looking at dir: " << dir << endl;
  
  CompactSequence seq;
  seq.load(dir);
  cout << "Loaded successfully" << endl;
  pcl::visualization::CloudViewer cloud_viewer("cloud");
  for(size_t i = 0; i < seq.size(); i++){
    cout << "Viewing cloud: " << i << endl;
    Cloud::Ptr cloud = seq.getCloud(i);
    cloud_viewer.showCloud(cloud);
    if(i < seq.size() ){
      double dt = seq.timestamps_[i+1]-seq.timestamps_[i];
      usleep(1e6*dt);
    }
  }
  
  return 0;
}

