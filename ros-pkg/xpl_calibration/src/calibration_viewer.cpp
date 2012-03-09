#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <eigen_extensions/eigen_extensions.h>
#include <rgbd_sequence/rgbd_sequence.h>
#include <rgbd_sequence/stream_sequence.h>

using namespace std;
using namespace rgbd;
using namespace pcl::visualization;

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
  
  StreamSequence::Ptr sseq0(new StreamSequence);
  StreamSequence::Ptr sseq1(new StreamSequence);
  sseq0->load(argv[1]);
  sseq1->load(argv[2]);
  sseq1->applyTimeOffset(sync(0));

  PCLVisualizer vis("Overlay");
  vis.setBackgroundColor(255, 255, 255);
  Cloud::Ptr transformed(new Cloud);
  transformPointCloud(*sseq1->getCloud(0), *transformed, transform);
  vis.addPointCloud(sseq0->getCloud(0), "pcd0");
  vis.addPointCloud(transformed, "pcd1");
  vis.setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, 3, "pcd0");
  vis.setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, 3, "pcd1");

  // -- Hardcode the camera for the Kinect.
  vis.camera_.clip[0] = 0.00387244;
  vis.camera_.clip[1] = 3.87244;
  vis.camera_.focal[0] = -0.160878;
  vis.camera_.focal[1] = -0.0444743;
  vis.camera_.focal[2] = 1.281;
  vis.camera_.pos[0] = 0.0402195;
  vis.camera_.pos[1] = 0.0111186;
  vis.camera_.pos[2] = -1.7;
  vis.camera_.view[0] = 0;
  vis.camera_.view[1] = -1;
  vis.camera_.view[2] = 0;
  vis.camera_.window_size[0] = 1678;
  vis.camera_.window_size[1] = 525;
  vis.camera_.window_pos[0] = 2;
  vis.camera_.window_pos[1] = 82;
  vis.updateCamera();
  
  double thresh = 0.015;
  ROS_WARN_STREAM("Showing clouds dt of less than " << thresh << ".");
  for(size_t i = 0; i < sseq0->size(); ++i) {
    Cloud::Ptr overlay = sseq0->getCloud(i);
    double ts0 = overlay->header.stamp.toSec();
    double dt;
    Cloud::Ptr pcd1 = sseq1->getCloud(ts0, &dt);
    Cloud::Ptr transformed(new Cloud);
    cout << "dt = " << dt << endl;
    if(dt > thresh)
      continue;

    transformPointCloud(*pcd1, *transformed, transform);
    vis.updatePointCloud(sseq0->getCloud(i), "pcd0");
    vis.updatePointCloud(pcd1, "pcd1");
    vis.spinOnce(15);
    if(vis.wasStopped())
      break;
  }

  return 0;
}
