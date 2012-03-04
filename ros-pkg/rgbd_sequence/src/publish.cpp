#include <rgbd_sequence/publisher.h>

using namespace rgbd;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rgbd_publisher");
  Publisher pub;
  pub.run();
  
  return 0;
}

