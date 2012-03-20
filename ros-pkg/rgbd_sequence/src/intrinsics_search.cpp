#include <rgbd_sequence/tube_measurer.h>
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
  oss << "Usage: intrinsics_search SEQ" << endl;
  oss << "  SEQ is a sequence made for this purpose." << endl;
  return oss.str();
}


int main(int argc, char** argv)
{
  if(argc != 2) {
    cout << usageString() << endl;
    return -1;
  }
  
  TubeMeasurer tm;
  tm.run(argv[1]);

  return 0;
}
