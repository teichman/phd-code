#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <eigen_extensions/eigen_extensions.h>
#include <rgbd_sequence/rgbd_sequence.h>

using namespace std;


string usageString()
{
  ostringstream oss;
  oss << "Usage: calibration_viewer SEQ SEQ CAL" << endl;
  oss << "  where SEQ is a RGBDSequence and CAL is a 4x4 .eig.txt file that describes" << endl;
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
  
  RGBDSequence::Ptr reference(new RGBDSequence);
  RGBDSequence::Ptr target(new RGBDSequence);
  reference->load(argv[1]);
  target->load(argv[2]);
    
  RGBDCloud::Ptr overlay(new RGBDCloud);
  RGBDCloud::Ptr transformed(new RGBDCloud);
  *overlay = *reference->pcds_[0];
  Eigen::Affine3f transform(mat);
  transformPointCloud(*target->pcds_[0], *transformed, transform);
  *overlay += *transformed;

  pcl::visualization::CloudViewer vis("Overlay");
  vis.showCloud(transformed);
  cin.ignore();
  vis.showCloud(reference->pcds_[0]);
  cin.ignore();
  vis.showCloud(overlay);
  cin.ignore();
  
  return 0;
}
