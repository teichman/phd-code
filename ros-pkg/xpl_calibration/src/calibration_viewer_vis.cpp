#define BOOST_FILESYSTEM_VERSION 2
#include <boost/filesystem.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <eigen_extensions/eigen_extensions.h>
#include <rgbd_sequence/rgbd_sequence.h>
#include <rgbd_sequence/stream_sequence.h>
#include <opencv2/calib3d/calib3d.hpp>

enum Mode_t{
  ALL, REF, TARGET, GROUND
};

using namespace std;
using namespace rgbd;
namespace bfs = boost::filesystem;

string usageString()
{
  ostringstream oss;
  oss << "Usage: calibration_viewer SEQ SEQ CAL SYNC [OUTFOLDER] [--checker] [CAL_GROUND]"  << endl;
  oss << "  where SEQ is a Sequence," << endl;
  oss << "  CAL is a 4x4 .eig.txt file that describes" << endl;
  oss << "    the transform that brings the second sequence to the coordinate system that the" << endl;
  oss << "    first lives in," << endl;
  oss << "  SYNC is a 1x1 .eig.txt file with the time offset to add to the timestamps of the second SEQ." << endl;
  oss << "  CAL_GROUND is the corresponding ground truth transformation matrix" << endl;
  return oss.str();
}

string fname;

void viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
{
       viewer.setBackgroundColor(1.0,1.0,1.0);
       viewer.saveScreenshot(fname);
       std::cout << "Image has been saved as" << fname << std::endl;
       viewer.setBackgroundColor(0,0,0);
}

int main(int argc, char** argv)
{
  if(argc != 5 && argc != 6 && argc != 7) {
    cout << usageString() << endl;
    return 0;
  }

  Eigen::Matrix4f mat;
  eigen_extensions::loadASCII(argv[3], &mat);
  cout << "Loaded transform: " << endl;
  cout << mat << endl;
  Eigen::Affine3f transform(mat);
  bool use_ground = false;
  Eigen::Affine3f ground_transform;
  bool use_checker = false;
  string dir = ".";
  if(argc >= 6){
    dir = argv[5];
  }
  if(argc == 7){
    string arg = argv[6];
    if (arg == "--checker")
      use_checker = true;
    else{
      use_ground = true;
      Eigen::Matrix4f mat;
      eigen_extensions::loadASCII(arg, &mat);
      ground_transform = Eigen::Affine3f(mat);
    }
  }

  Eigen::VectorXd sync;
  eigen_extensions::loadASCII(argv[4], &sync);
  
  StreamSequence::Ptr sseq0(new StreamSequence);
  StreamSequence::Ptr sseq1(new StreamSequence);
  sseq0->load(argv[1]);
  sseq1->load(argv[2]);
  sseq1->applyTimeOffset(sync(0));

  pcl::visualization::CloudViewer vis("Overlay");
  double thresh = 0.05;
  ROS_WARN_STREAM("Showing clouds dt of less than " << thresh << ".");
  Mode_t view_mode = ALL;
  bool pause = false;
  for(size_t i = 0; i < sseq0->size(); ) {
    Cloud::Ptr overlay = sseq0->getCloud(i);
    cv::Mat3b img0 = sseq0->getImage(i);
    double ts0 = overlay->header.stamp.toSec();
    double dt;
    Cloud::Ptr pcd1 = sseq1->getCloud(ts0, &dt);
    cv::Mat3b img1 = sseq1->getImage(ts0, &dt);
    if(use_checker){
      vector<cv::Point2f> corners0,corners1;
      if(cv::findChessboardCorners(img0, cv::Size(8,6), corners0, cv::CALIB_CB_ADAPTIVE_THRESH+cv::CALIB_CB_FAST_CHECK))
        cv::drawChessboardCorners( img0, cv::Size(8,6), corners0, true );
      if(cv::findChessboardCorners(img1, cv::Size(8,6), corners1, cv::CALIB_CB_ADAPTIVE_THRESH+cv::CALIB_CB_FAST_CHECK))
        cv::drawChessboardCorners( img1, cv::Size(8,6), corners1, true );
    }
    Cloud::Ptr transformed(new Cloud);
    Cloud::Ptr ground_overlay;
    Cloud::Ptr ground_transformed(new Cloud);
    //cout << "dt = " << dt << endl;
    if(dt < thresh){
      transformPointCloud(*pcd1, *transformed, transform);
      if(use_ground)
        transformPointCloud(*pcd1, *ground_transformed, ground_transform);
    }
    *overlay += *transformed;
    if(use_ground){
      ground_overlay = sseq0->getCloud(i);
      *ground_overlay += *ground_transformed;
    }
    cv::imshow("Ref", img0);
    cv::imshow("Target", img1);
    char keypress = cv::waitKey(30);
    switch(keypress){
      case '1':
        view_mode = REF;
        break;
      case '2':
        view_mode = TARGET;
        break;
      case '3':
        view_mode = ALL;
        break;
      case '4':
        view_mode = GROUND;
        break;
      case '-':
        i--;
        break;
      case '=':
        i++;
        break;
      case ' ':
        pause = !pause;
        break;
      case 's':
        ROS_ASSERT(!bfs::exists(dir));
        bfs::create_directory(dir);
        cv::imwrite(dir+"/img_ref.png", img0);
        cv::imwrite(dir+"/img_target.png", img1);
        fname = dir+"/cloud_ref.png";
        vis.showCloud(sseq0->getCloud(i));
        vis.runOnVisualizationThreadOnce(viewerOneOff);
        vis.showCloud(sseq0->getCloud(i));
        //cout << "Modify view and save it" << endl;
        //cv::waitKey();
        fname = dir+"/cloud_target.png";
        vis.showCloud(transformed);
        vis.runOnVisualizationThreadOnce(viewerOneOff);
        vis.showCloud(transformed);
        fname = dir+"/cloud_combined.png";
        vis.showCloud(overlay);
        vis.runOnVisualizationThreadOnce(viewerOneOff);
        vis.showCloud(overlay);
        if(use_ground){
          fname = dir+"/cloud_combined_ground.png";
          vis.showCloud(ground_overlay);
          vis.runOnVisualizationThreadOnce(viewerOneOff);
          vis.showCloud(ground_overlay);
        }
        break;

    }
    switch(view_mode){
      case REF:
        vis.showCloud(sseq0->getCloud(i));
        break;
      case TARGET:
        vis.showCloud(transformed);
        break;
      case ALL:
        vis.showCloud(overlay);
        break;
      case GROUND:
        if(use_ground)
          vis.showCloud(ground_overlay);
        break;
    }
    if(!pause)
      i++;
    if(vis.wasStopped(30))
      break;
  }

  return 0;
}
