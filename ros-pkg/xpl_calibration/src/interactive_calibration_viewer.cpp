#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <eigen_extensions/eigen_extensions.h>
#include <rgbd_sequence/rgbd_sequence.h>
#include <rgbd_sequence/stream_sequence.h>

using namespace std;
using namespace rgbd;
using namespace Eigen;

string usageString()
{
  ostringstream oss;
  oss << "Usage: interactive_calibration_viewer SEQ SEQ CAL SYNC CAL_OUT"  << endl;
  oss << "  where SEQ is a Sequence," << endl;
  oss << "  CAL is a 4x4 .eig.txt file that describes" << endl;
  oss << "    the transform that brings the second sequence to the coordinate system that the" << endl;
  oss << "    first lives in," << endl;
  oss << "  SYNC is a 1x1 .eig.txt file with the time offset to add to the timestamps of the second SEQ." << endl;
  oss << "  CAL_OUT is the output, modified .eig.txt" << endl;
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

  pcl::visualization::CloudViewer vis("Overlay");
  cv::namedWindow("keycommand");
  //Compute some simple Eigen transforms
  Eigen::Affine3f moveLeft, moveRight, moveUp, moveDown, moveForward, moveBackward,
                  rollLeft, rollRight, pitchUp, pitchDown, yawLeft, yawRight;
  dx = 0.001;
  da = M_PI/100;
  moveLeft = Translation3f(-dx,0,0);
  moveRight = Translation3f(dx,0,0);
  moveUp = Translation3f(0,dx,0);
  moveDown = Translation3f(0,-dx,0);
  moveForward = Translation3f(0,0,dx);
  moveBackward = Translation3f(0,0,-dx);
  rollLeft = AngleAxisf(da, Vector3f(0,0,1));
  rollRight = AngleAxisf(-da, Vector3f(0,0,1));
  pitchUp = AngleAxisf(da, Vector3f(1,0,0));
  pitchDown = AngleAxisf(-da, Vector3f(1,0,0));
  yawLeft = AngleAxisf(da, Vector3f(0,1,0));
  yawRight = AngleAxisf(-da, Vector3f(0,1,0));
  bool quit = false;
  bool update_frame = true, update_transform = true;
  int frame = 0;
  double dt;
  Cloud::Ptr cloud0, cloud1;
  pcl::visualization::CloudViewer vis("Overlay");
  while(!quit){
    cout << "Transformation: " << transform.matrix() << endl;
    if(update_frame){
      cloud0 = sseq0->getCloud(frame);    
      cloud1 = sseq->getCloud(cloud->header.stamp.toSec(), dt);
    }
    if(update_frame || update_transform){
      Cloud::Ptr overlay(new Cloud);
      *overlay = *cloud0;
      Cloud::Ptr transformed(new Cloud);
      cout << "dt = " << dt << ednl;
      if(dt < 0.05)
        transformPointCloud(*cloud1, *transformed, transform);
      *overlay += *transformed;
      vis.showCloud(overlay)
        usleep(10 * 1000);
    }
    update_frame = false;
    update_transform = false;
    char cmd = cv::waitKey();
    switch(cmd)
    {
      case 'a':
        transform = moveLeft*transform;
        cout << "Moving left by " << dx << endl;
        update_transform = true;
        break;
      case 'd':
        transform = moveRight*transform;
        cout << "Moving right by " << dx << endl;
        update_transform = true;
        break;
      case 'w':
        transform = moveUp*transform;
        cout << "Moving up by " << dx << endl;
        update_transform = true;
        break;
      case 's':
        transform = moveDown*transform;
        cout << "Moving down by " << dx << endl;
        downdate_transform = true;
        break;
      case 'e':
        transform = moveForward*transform;
        cout << "Moving forward by " << dx << endl;
        forwarddate_transform = true;
        break;
      case 'q':
        transform = moveBackward*transform;
        cout << "Moving back by " << dx << endl;
        update_transform = true;
        break;
      case 'j':
        transform = rollLeft*transform;
        cout << "Rolling left by " << da << endl;
        update_transform = true;
        break;
      case 'l':
        transform = rollRight*transform;
        cout << "Rolling right by " << da << endl;
        update_transform = true;
        break;
      case 'i':
        transform = pitchDown*transform;
        cout << "Pitching down by " << da << endl;
        update_transform = true;
        break;
      case 'k':
        transform = pitchUp*transform;
        cout << "Pitching up by " << da << endl;
        update_transform = true;
        break;
      case 'u':
        transform = yawLeft*transform;
        cout << "Yawing left by " << da << endl;
        update_transform = true;
        break;
      case 'o':
        transform = yawRight*transform;
        cout << "Yawing right by " << da << endl;
        update_transform = true;
        break;
      case '=':
        if(frame+1 < sseq0->size()){
          frame++;
          update_frame = true;
        }
        break;
      case '-':
        if(frame-1 >= 0){
          frame--;
          update_frame = true;
        }
        break;
      case '1':
        quit = true;
        break;
    }

  }

  return 0;
}
