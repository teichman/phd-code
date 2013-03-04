#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <eigen_extensions/eigen_extensions.h>
#include <rgbd_sequence/rgbd_sequence.h>
#include <rgbd_sequence/stream_sequence_base.h>

using namespace std;
using namespace rgbd;

string usageString()
{
  ostringstream oss;
  oss << "Usage: calibration_viewer SEQ SEQ CAL SYNC"  << endl;
  oss << "  where SEQ is a Sequence," << endl;
  oss << "  CAL is a 4x4 .eig.txt file that describes" << endl;
  oss << "    the transform that brings the second sequence to the coordinate system that the" << endl;
  oss << "    first lives in," << endl;
  oss << "  SYNC is a 1x1 .eig.txt file with the time offset to add to the timestamps of the second SEQ." << endl;
  return oss.str();
}

int main(int argc, char** argv)
{
  if(argc != 5) {
    cout << usageString() << endl;
    return 0;
  }

  Eigen::Matrix4f mat;
  eigen_extensions::loadASCII(argv[3], &mat);
  cout << "Loaded transform: " << endl;
  cout << mat << endl;
  Eigen::Affine3f transform(mat);

  Eigen::VectorXd sync;
  eigen_extensions::loadASCII(argv[4], &sync);
 
  StreamSequenceBase::Ptr sseq0 = StreamSequenceBase::initializeFromDirectory (argv[1]);
  StreamSequenceBase::Ptr sseq1 = StreamSequenceBase::initializeFromDirectory (argv[2]);
  sseq1->applyTimeOffset(sync(0));

  pcl::visualization::CloudViewer vis("Overlay");
  double thresh = 0.015;
  ROS_WARN_STREAM("Showing clouds dt of less than " << thresh << ".");
  for(size_t i = 0; i < sseq0->size(); ++i) {
    Cloud::Ptr overlay = sseq0->getCloud(i);
    double ts0 = overlay->header.stamp.toSec();
    double dt;
    Cloud::Ptr pcd1 = sseq1->getCloud(ts0, &dt);
    Cloud::Ptr transformed(new Cloud);
    cout << "dt = " << dt << endl;
    if(fabs(dt) > thresh)
      continue;

    
    transformPointCloud(*pcd1, *transformed, transform);
    *overlay += *transformed;
    vis.showCloud(overlay);
    if(vis.wasStopped(30))
      break;
  }

  return 0;
}
