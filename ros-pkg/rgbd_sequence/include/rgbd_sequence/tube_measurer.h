#ifndef TUBE_MEASURER_H
#define TUBE_MEASURER_H

#include <rgbd_sequence/vis_wrapper.h>
#include <rgbd_sequence/stream_sequence.h>
#include <rgbd_sequence/projector.h>

namespace rgbd
{

  class TubeMeasurer
  {
  public:
    TubeMeasurer();
    void run(const std::string& path);

  protected:
    VisWrapper vw_;
    StreamSequence sseq_;
    int idx_;
    std::vector<Point> selected_;
    Projector proj_;
    Cloud::Ptr warped_;
    Frame frame_;
    
    void increment(int num);
    void incrementIntrinsics(double dfx, double dfy, double dcx, double dcy);
    void findTube(const rgbd::Cloud& pcd);
    void pointPickingCallback(const pcl::visualization::PointPickingEvent& event, void* cookie);
  };

}

#endif // TUBE_MEASURER_H
