#include <jarvis/detection_visualizer.h>

using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visualize_detections");
  DetectionVisualizer dv(320, 240);
  ros::spin();
  
  return 0;
}
