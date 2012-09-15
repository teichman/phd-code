#include <xpl_calibration/frame_aligner.h>

using namespace std;
using namespace Eigen;
using namespace rgbd;

FrameAligner::FrameAligner(const rgbd::PrimeSenseModel& model0,
			   const rgbd::PrimeSenseModel& model1,
			   double max_range,
			   GridSearchViewHandler* view_handler) :
  max_range_(max_range),
  model0_(model0),
  model1_(model1),
  view_handler_(view_handler)
{
}

Eigen::Affine3d FrameAligner::align(rgbd::Frame frame0, rgbd::Frame frame1,
				    double* count, double* final_mde) const
{
  // -- Run grid search.
  ScopedTimer st("FrameAligner::align");
  FrameAlignmentMDE::Ptr mde(new FrameAlignmentMDE(model0_, frame0, model1_, frame1, max_range_, 0.25));
  GridSearch gs(6);
  gs.verbose_ = false;
  gs.view_handler_ = view_handler_;
  gs.objective_ = mde;
  gs.num_scalings_ = 5;
  double max_res_rot = 1.5 * M_PI / 180.0;
  double max_res_trans = 0.05;
  gs.max_resolutions_ << max_res_rot, max_res_rot, max_res_rot, max_res_trans, max_res_trans, max_res_trans;
  int gr = 2;
  gs.grid_radii_ << gr, gr, gr, gr, gr, gr;
  double sf = 0.5;
  gs.scale_factors_ << sf, sf, sf, sf, sf, sf;
  gs.couplings_ << 0, 1, 2, 1, 0, 3;  // Search over (pitch, y) and (yaw, x) jointly.
  ArrayXd x = gs.search(ArrayXd::Zero(6));

  // -- Print out statistics.
  cout << "============================== Frame alignment complete" << endl;
  cout << "GridSearch solution: " << x.transpose() << endl;
  cout << "Computed " << gs.num_evals_ << " evals in " << gs.time_ << " seconds." << endl;
  cout << gs.num_evals_ / gs.time_ << " evals / second." << endl;
  cout << gs.time_ / gs.num_evals_ << " seconds / eval." << endl;

  mde->count_ = count;
  *final_mde = mde->eval(x);
  
  // vector<ArrayXd> xs = gs.x_history_.back();
  // ArrayXd vals = gs.obj_history_.back();
  // cout << setw(20) << "Delta objective\t" << setw(30) << "Delta x" << endl;
  // cout << setiosflags(ios::right) << setiosflags(ios::fixed) << setprecision(7) << setw(13);
  // for(size_t i = 0; i < xs.size(); ++i) {
  //   cout << setiosflags(ios::right) << setiosflags(ios::fixed) << setprecision(7) << setw(13) << vals(i) - *final_mde << "\t";
  //   ArrayXd dx = xs[i] - x;
  //   for(int j = 0; j < xs[i].rows(); ++j)
  //     cout << setiosflags(ios::right) << setiosflags(ios::fixed) << setprecision(7) << setw(13) << dx(j) << " ";
  //   cout << endl;
  // }

  cout << " -- Single-number statistics" << endl;
  cout << "Final MDE: " << *final_mde << endl;
  cout << "Count: " << *count << endl;
  // double min_mde_grid = numeric_limits<double>::max();
  // for(int i = 0; i < vals.rows(); ++i)
  //   min_mde_grid = min(vals(i) - *final_mde, min_mde_grid);
  // cout << "Minimum delta MDE in the last grid: " << min_mde_grid << endl;
  cout << "==============================" << endl;
  
  return generateTransform(x(0), x(1), x(2), x(3), x(4), x(5)).cast<double>();
}

