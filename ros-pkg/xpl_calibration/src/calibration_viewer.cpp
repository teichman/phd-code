#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <eigen_extensions/eigen_extensions.h>
#include <rgbd_sequence/rgbd_sequence.h>
#include <rgbd_sequence/stream_sequence.h>

using namespace std;
using namespace rgbd;

string usageString()
{
  ostringstream oss;
  oss << "Usage: calibration_viewer SEQ SEQ CAL" << endl;
  oss << "  where SEQ is a Sequence and CAL is a 4x4 .eig.txt file that describes" << endl;
  oss << "  the transform that brings the second sequence to the coordinate system that the" << endl;
  oss << "  first lives in." << endl;
  return oss.str();
}

int main(int argc, char** argv)
{
  if(argc != 4) {
    cout << usageString() << endl;
    return 0;
  }

  Eigen::Matrix4f mat;
  eigen_extensions::loadASCII(argv[3], &mat);
  cout << "Loaded transform: " << endl;
  cout << mat << endl;
  Eigen::Affine3f transform(mat);
  
  StreamSequence::Ptr sseq0(new StreamSequence);
  StreamSequence::Ptr sseq1(new StreamSequence);
  sseq0->load(argv[1]);
  sseq1->load(argv[2]);

  pcl::visualization::CloudViewer vis("Overlay");
  double thresh = 0.1;
  for(size_t i = 0; i < sseq0->size(); ++i) {
    Cloud::Ptr overlay = sseq0->getCloud(i);
    double ts0 = overlay->header.stamp.toSec();
    double dt;
    Cloud::Ptr pcd1 = sseq1->getCloud(ts0, &dt);
    Cloud::Ptr transformed(new Cloud);
    if(dt < thresh)
      transformPointCloud(*pcd1, *transformed, transform);
    *overlay += *transformed;
    vis.showCloud(overlay);
    usleep(30 * 1000);
  }

  return 0;
}
