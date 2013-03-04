#include <xpl_calibration/mocap_detector.h>
#include <pcl/visualization/cloud_viewer.h>
using namespace std;

string usageString()
{
  ostringstream oss;
  oss << "Usage: mocap_test SEQ" << endl;
  oss << "Where SEQ is the reference StreamSequence" << endl;
  return oss.str();
}

int main(int argc, char** argv)
{
  if(argc != 2) {
    cout << usageString() << endl;
    return 0;
  }
  string seq_dir = argv[1];
  StreamSequenceBase::Ptr seq = StreamSequenceBase::initializeFromDirectory (seq_dir);
  pcl::visualization::CloudViewer vis("Cloud");
  MocapDetector md;
  for(size_t i = 0; i < seq->size(); i++){
    Point tl, tr, bl, br;
    if(! md.locatePoints(seq, i, tl, tr, bl, br ) )
      continue;
    Cloud::Ptr pcd = seq->getCloud(i);
    pcd->points.push_back(tl);
    pcd->points.push_back(tr);
    pcd->points.push_back(bl);
    pcd->points.push_back(br);
    vis.showCloud(pcd);
    cv::imshow("Image", seq->getImage(i));
    cv::waitKey();
  }
  return 0;
}
