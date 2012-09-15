#include <slam_interface/publisher.h>
#include <rgbd_sequence/stream_sequence.h>

using namespace rgbd;
using namespace slam_interface;
using namespace std;

string usageString(char** argv)
{
  ostringstream oss;
  oss << "Usage: " << argv[0] << " SEQ PAUSETIME MAXDIST SUBSAMPLE" << endl
      << " where SEQ is a stream_sequence" << endl
      << " where PAUSETIME is the amount of time to pause between frames" << endl
      << " where MAXDIST is the maximum allowed distance from the camera" << endl
      << " where SUBSAMPLE is the number of frames to drop (1 = keep every frame, 2 = keep every other frame, etc)" << endl
      << "Publishes the frames of a stream_sequence to image_out and depth_out" << endl;
  return oss.str();
}

int main(int argc, char** argv)
{
  if(argc < 5)
  {
    cout << usageString(argv);
    return 1;
  }
  string seq_dir = argv[1];
  ros::init(argc, argv, "rgbd_publisher");
  Publisher pub;
  StreamSequence::Ptr seq(new StreamSequence);
  seq->load(seq_dir);
  pub.pausetime_ = atof(argv[2]); // seconds to wait
  pub.maxdist_ = atof(argv[3]); // max distance
  pub.subsample_ = atoi(argv[4]); // amount to subsample frames by
  pub.run(seq);
  cout << "Done running!" << endl;
  
  return 0;
}

