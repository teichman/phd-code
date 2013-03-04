#ifndef INTRINSICS_VISUALIZER_H
#define INTRINSICS_VISUALIZER_H

#include <rgbd_sequence/vis_wrapper.h>
#include <rgbd_sequence/stream_sequence_base.h>
#include <rgbd_sequence/primesense_model.h>
#include <optimization/grid_search.h>

namespace rgbd
{

  class AcceptedPoints : public std::vector< std::pair<ProjectivePoint, ProjectivePoint> >,
                         public Serializable
  {
    void serialize(std::ostream& out) const;
    void deserialize(std::istream& in);
  };
  
  class IntrinsicsVisualizer
  {
  public:
    IntrinsicsVisualizer();
    void run(const std::string& path, double actual_distance);

  protected:
    VisWrapper vw_;
    StreamSequenceBase::Ptr sseq_;
    int idx_;
    //! The currently selected points.  Zero or one.
    std::vector<Point> selected_;
    std::vector< std::vector<Point> > visible_pairs_;
    AcceptedPoints accepted_;
    Cloud::Ptr pcd_;
    Frame frame_;
    std::string path_;
    double actual_distance_;
    PrimeSenseModel model_;
    bool show_color_;

    void decolorize(rgbd::Cloud* pcd) const;
    void increment(int num);
    void incrementIntrinsics(double dfx, double dfy, double dcx, double dcy);
    void findTube(const rgbd::Cloud& pcd);
    void pointPickingCallback(const pcl::visualization::PointPickingEvent& event, void* cookie);
    void clearSelection();
    void acceptVisible();
    void saveAccepted() const;
    void loadAccepted();
    void gridSearch();
    void printErrors(const Eigen::ArrayXd& x) const;
  };

  class LossFunction : public ScalarFunction
  {
  public:
    typedef boost::shared_ptr<LossFunction> Ptr;
    
    LossFunction(const AcceptedPoints& accepted, double actual_distance, int width, int height);
    //! fx, fy, cx, cy.
    //! Returns average distance error between points for the given intrinsics.
    double eval(const Eigen::VectorXd& x) const;
    
  protected:
    AcceptedPoints accepted_;
    double actual_distance_;
    int width_;
    int height_;
  };
  
}

#endif // INTRINSICS_VISUALIZER_H
