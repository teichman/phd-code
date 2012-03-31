#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <eigen_extensions/eigen_extensions.h>
#include <rgbd_sequence/stream_sequence.h>

using namespace std;
using namespace rgbd;

string usageString()
{
  ostringstream oss;
  oss << "Usage: offset_maker SEQ SEQ SYNC_OUT [SYNC_IN]"  << endl;
  oss << "  where SEQ is a Sequence," << endl;
  oss << "  SYNC is the output 1x1 .eig.txt file with the time offset to add to"
      << " the timestamps of the second SEQ." << endl;
  return oss.str();
}

int main(int argc, char** argv)
{
  if(argc != 4 && argc != 5) {
    cout << usageString() << endl;
    return 0;
  }

  Eigen::VectorXd sync;
  if(argc == 5)
    eigen_extensions::loadASCII(argv[4], &sync);
  else
    sync = Eigen::VectorXd::Constant(1,0);
  
  StreamSequence::Ptr sseq0(new StreamSequence);
  StreamSequence::Ptr sseq1(new StreamSequence);
  sseq0->load(argv[1]);
  sseq1->load(argv[2]);
  sseq1->applyTimeOffset(sync(0));

  double thresh = 0.015;
  ROS_WARN_STREAM("Showing images with dt of less than " << thresh << ".");
  int i = 0;
  while(true){
    cout << "Current offset: " << sync(0) << endl;
    cv::Mat3b image0 = sseq0->getImage(i);
    double ts0 = sseq0->timestamps_[i];
    cout << "ts0: " << setprecision(16) << ts0 << endl;
    double dt;
    cv::Mat3b image1 = sseq1->getImage(ts0, &dt);
    cout << "dt = " << dt << endl;
    cv::imshow("Image0", image0);
    if(fabs(dt) < thresh){
      cv::imshow("Image1", image1);
    } else{
      cv::imshow("Image1", cv::Mat3b::zeros(480,640) );
    }
    char inp = cv::waitKey(0);
    double off_add;
    switch(inp){
      case '=':
        //Increase dt by 1 milisecond
        off_add = 0.005;
        sseq1->applyTimeOffset(off_add);
        sync(0) += off_add;
        cout << "Increased offset by " << off_add << endl; 
        break;
      case '-':
        //Decrease dt by 1 milisecond
        off_add = -0.005;
        sseq1->applyTimeOffset(off_add);
        sync(0) += off_add;
        cout << "Decreased offset by " << off_add << endl; 
        break;
      case 'd':
        if(i < (int)sseq0->size()-1)
          i += 1;
        break;
      case 'a':
        if(i > 0)
          i -= 1;
        break;
      case 's':
        cout << "Saving to " << argv[3] << endl;
        eigen_extensions::saveASCII(sync, argv[3]);
        return 0;
        break;
      case 'q':
        return 0;
        break;
    }

  }

  return 0;
}

