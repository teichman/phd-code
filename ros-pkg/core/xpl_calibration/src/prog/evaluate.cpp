#include <Eigen/Geometry>
#include <iostream>
#include <eigen_extensions/eigen_extensions.h>
#include <rgbd_sequence/stream_sequence_base.h>
#include <pcl/common/transforms.h>

using namespace std;
using namespace Eigen;

string usageString()
{
  ostringstream oss;
  oss << "Usage: evaluate GROUND_TRUTH_TRANS ESTIMATED_TRANS GROUND_TRUTH_SYNC ESTIMATED_SYNC SSEQ_SRC SSEQ_TGT" << endl;
  return oss.str();
}

int main(int argc, char** argv)
{
  if(argc < 6) {
    cout << usageString() << endl;
    return 0;
  }

  MatrixXd gtt;
  MatrixXd estt;
  eigen_extensions::loadASCII(argv[3], &gtt);
  eigen_extensions::loadASCII(argv[4], &estt);
  cout << "Ground truth sync: " << gtt(0) << endl;
  cout << "Estimated sync: " << estt(0) << endl;
  cout << "Sync error: " << fabs(gtt(0) - estt(0)) << endl;

  Matrix4d gt;
  Matrix4d est;
  cout << "Loading gt" << endl;
  eigen_extensions::loadASCII(argv[1], &gt);
  cout << "Loading estimated" << endl;
  eigen_extensions::loadASCII(argv[2], &est);
  cout << "Ground truth: " << endl << gt << endl;
  cout << endl;
  cout << "Estimated: " << endl << est << endl;

  cout << "Frobenius: " << (gt - est).norm() << endl;

  Vector3d Tgt = gt.block<3, 1>(0, 3);
  Vector3d Test = est.block<3, 1>(0, 3);
  cout << "Euclidean distance: " << (Tgt - Test).norm() << endl;
  
  Matrix3d Rgt = gt.block<3, 3>(0, 0);
  Matrix3d Rest = est.block<3, 3>(0, 0);
  AngleAxisd aa(Rest.inverse() * Rgt);
  cout << "Angle: " << aa.angle() << endl;
  cout << "Axis: " << aa.axis().transpose() << endl;
  
  AngleAxisd final(AngleAxisd(M_PI/2.0, Vector3d::UnitZ()) * AngleAxisd(M_PI/2.0, Vector3d::UnitY()));
  cout << "jesse: " << final.angle() << endl;

  // Load the source, transform it by both, see how far points are off from one another
  rgbd::StreamSequenceBase::ConstPtr sseq = rgbd::StreamSequenceBase::initializeFromDirectory (argv[5]);
  float avg_error = 0;
  int npt = 0;
  for (size_t i = 0; i < sseq->size (); i++)
  {
    cout << "Aggregating errors from frame " << i+1 << "/" << sseq->size () << endl;
    rgbd::Cloud::ConstPtr cloud_orig = sseq->at (i);
    rgbd::Cloud cloud_gt, cloud_est;
    pcl::transformPointCloud (*cloud_orig, cloud_gt, gt);
    pcl::transformPointCloud (*cloud_orig, cloud_est, est);
    for (size_t j = 0; j < cloud_gt.size (); j++)
    {
      if (pcl_isnan(cloud_orig->at (j).z) || cloud_orig->at (j).z > 5)
        continue;
      float error = (cloud_gt[j].getVector3fMap () - cloud_est[j].getVector3fMap ()).norm ();
      avg_error += (error - avg_error) / static_cast<float> (++npt);
    }
  }
  cout << "avg point to point error: " << avg_error << endl;

  return 0;
}
