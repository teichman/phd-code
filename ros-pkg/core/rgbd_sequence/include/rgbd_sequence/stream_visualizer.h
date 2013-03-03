#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/highgui/highgui.hpp>
#include <rgbd_sequence/stream_sequence.h>
#include <bag_of_tricks/agent.h>

namespace rgbd
{
  
  class StreamVisualizer : public Agent
  {
  public:
    StreamVisualizer(StreamSequence::ConstPtr sseq);
    virtual ~StreamVisualizer() {}
    virtual void _run();

  protected:
    pcl::visualization::PCLVisualizer vis_;
    StreamSequence::ConstPtr sseq_;
    int idx_;
    Cloud::Ptr pcd_;
    bool needs_update_;
  
    virtual void increment(int num);
    virtual void handleKeypress(char key);

    // These get registered with the PCLVisualizer in the constructor
    // and so should not be overridden.
    void pointPickingCallback(const pcl::visualization::PointPickingEvent& event, void* cookie);
    void keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* cookie);
  };

}
