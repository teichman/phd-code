#include <xpl_calibration/frame_aligner.h>

using namespace std;
using namespace Eigen;
using namespace rgbd;

FrameAligner::FrameAligner(const rgbd::PrimeSenseModel& model0,
			   const rgbd::PrimeSenseModel& model1,
			   GridSearchViewHandler* view_handler) :
  model0_(model0),
  model1_(model1),
  view_handler_(view_handler)
{
}

Eigen::Affine3d FrameAligner::align(rgbd::Frame frame0, rgbd::Frame frame1) const
{
  FrameAlignmentMDE::Ptr mde(new FrameAlignmentMDE(model0_, frame0, model1_, frame1));
  //mde->point_utilization_ = 0.0001;
  
  GridSearch gs(6);
  gs.verbose_ = false;
  gs.view_handler_ = view_handler_;
  gs.objective_ = mde;
  gs.num_scalings_ = 5;
  double max_res_rot = 2.0 * M_PI / 180.0;
  double max_res_trans = 0.05;
  gs.max_resolutions_ << max_res_rot, max_res_rot, max_res_rot, max_res_trans, max_res_trans, max_res_trans;
  int gr = 3;
  gs.grid_radii_ << gr, gr, gr, gr, gr, gr;
  double sf = 0.5;
  gs.scale_factors_ << sf, sf, sf, sf, sf, sf;
  gs.couplings_ << 0, 1, 2, 1, 0, 3;  // Search over (pitch, y) and (yaw, x) jointly.
  
  ArrayXd x = gs.search(ArrayXd::Zero(6));
  cout << "GridSearch solution: " << x.transpose() << endl;

  return generateTransform(x(0), x(1), x(2), x(3), x(4), x(5)).cast<double>();
}

