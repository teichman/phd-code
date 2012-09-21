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

bool FrameAligner::align(rgbd::Frame frame0, rgbd::Frame frame1,
			 const std::vector<cv::Point2d>& keypoints0, const std::vector<cv::Point2d>& keypoints1,
			 bool consider_wide_search, Eigen::Affine3d* f0_to_f1) const
{
  // -- Try to get rough transform between the two based on the corresponding keypoints.
  bool found_rough_transform = false;
  Affine3d guess = Affine3d::Identity();

  // -- Run grid search as desired.
  if(found_rough_transform)
    return confidentAlign(frame0, frame1, keypoints0, keypoints1, guess, f0_to_f1);
  else if(consider_wide_search)
    return unconfidentAlign(frame0, frame1, keypoints0, keypoints1, f0_to_f1);
  else
    return false;
}

bool FrameAligner::unconfidentAlign(rgbd::Frame frame0, rgbd::Frame frame1,
				    const std::vector<cv::Point2d>& keypoints0, const std::vector<cv::Point2d>& keypoints1,
				    Eigen::Affine3d* f0_to_f1) const
{
  // -- Run grid search.
  ScopedTimer st("FrameAligner::align");
  FrameAlignmentMDE::Ptr mde(new FrameAlignmentMDE(model0_, model1_, frame0, frame1, keypoints0, keypoints1, max_range_, 0.25));
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

  double count;
  mde->count_ = &count;
  double final_mde = mde->eval(x);
  
  cout << " -- Single-number statistics" << endl;
  cout << "Final MDE: " << final_mde << endl;
  cout << "Count: " << count << endl;
  cout << "==============================" << endl;

  // TODO: Add better conditions for alignment success.
  if(count < 20000 || final_mde > 0.5)
    return false;

  *f0_to_f1 = generateTransform(x(0), x(1), x(2), x(3), x(4), x(5)).cast<double>();
  return true;
}

bool FrameAligner::confidentAlign(rgbd::Frame frame0, rgbd::Frame frame1,
				  const std::vector<cv::Point2d>& keypoints0, const std::vector<cv::Point2d>& keypoints1, 
				  const Eigen::Affine3d& guess,
				  Eigen::Affine3d* f0_to_f1) const
{
  // -- Run grid search.
  ScopedTimer st("FrameAligner::align");
  FrameAlignmentMDE::Ptr mde(new FrameAlignmentMDE(model0_, model1_, frame0, frame1, keypoints0, keypoints1, max_range_, 0.25));
  GridSearch gs(6);
  gs.verbose_ = false;
  gs.view_handler_ = view_handler_;
  gs.objective_ = mde;
  gs.num_scalings_ = 5;
  double max_res_rot = 1.5 * M_PI / 180.0;
  double max_res_trans = 0.02;
  gs.max_resolutions_ << max_res_rot, max_res_rot, max_res_rot, max_res_trans, max_res_trans, max_res_trans;
  int gr = 1;
  gs.grid_radii_ << gr, gr, gr, gr, gr, gr;
  double sf = 0.5;
  gs.scale_factors_ << sf, sf, sf, sf, sf, sf;
  gs.couplings_ << 0, 1, 2, 1, 0, 3;  // Search over (pitch, y) and (yaw, x) jointly.
  //Convert guess to XYZRPY
  double rx, ry, rz, tx, ty, tz;
  generateXYZYPR(guess.cast<float>(), rx, ry, rz, tx, ty, tz);
  //Now search with that initialization
  Eigen::ArrayXd init(6); init << rx, ry, rz, tx, ty, tz;
  Eigen::ArrayXd x = gs.search(init);

  // -- Print out statistics.
  cout << "============================== Frame alignment complete" << endl;
  cout << "GridSearch solution: " << x.transpose() << endl;
  cout << "Computed " << gs.num_evals_ << " evals in " << gs.time_ << " seconds." << endl;
  cout << gs.num_evals_ / gs.time_ << " evals / second." << endl;
  cout << gs.time_ / gs.num_evals_ << " seconds / eval." << endl;

  double count;
  mde->count_ = &count;
  double final_mde = mde->eval(x);
  
  cout << " -- Single-number statistics" << endl;
  cout << "Final MDE: " << final_mde << endl;
  cout << "Count: " << count << endl;
  cout << "==============================" << endl;

  // TODO: Add better conditions for alignment success.
  if(count < 20000 || final_mde > 0.5)
    return false;
  
  *f0_to_f1 = generateTransform(x(0), x(1), x(2), x(3), x(4), x(5)).cast<double>();
  return true;
}

