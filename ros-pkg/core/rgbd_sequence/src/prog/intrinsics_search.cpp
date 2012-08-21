#include <rgbd_sequence/intrinsics_visualizer.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/ransac.h>
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
  oss << "Usage: intrinsics_search SEQ DISTANCE" << endl;
  oss << "  SEQ is a sequence made for this purpose." << endl;
  oss << "  DISTANCE is the ground truth distance between two reference points." << endl;
  return oss.str();
}


int main(int argc, char** argv)
{
  if(argc != 3) {
    cout << usageString() << endl;
    return -1;
  }
  
  IntrinsicsVisualizer tm;
  tm.run(argv[1], atof(argv[2]));

  return 0;
}
