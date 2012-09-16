#include <xpl_calibration/velo_to_asus_calibrator.h>

using namespace std;
using namespace Eigen;
using namespace rgbd;

VeloToAsusCalibration::VeloToAsusCalibration() :
  offset_(0)
{
  setVeloToAsus(Affine3f::Identity());
}

void VeloToAsusCalibration::serialize(std::ostream& out) const
{
  out.write((const char*)&offset_, sizeof(double));
  eigen_extensions::serialize(velo_to_asus_.matrix(), out);
}

void VeloToAsusCalibration::deserialize(std::istream& in)
{
  in.read((char*)&offset_, sizeof(double));
  Matrix4f mat;
  eigen_extensions::deserialize(in, &mat);
  setVeloToAsus(Affine3f(mat));  // Also sets asus_to_velo_.
}

std::string VeloToAsusCalibration::status(const std::string& prefix) const
{
  ostringstream oss;
  oss << prefix << "sync offset: " << offset_ << endl;
  oss << prefix << "transform: " << endl << velo_to_asus_.matrix() << endl;
  return oss.str();
}

VeloToAsusCalibrator::VeloToAsusCalibrator(const rgbd::PrimeSenseModel& model, GridSearchViewHandler* vis) :
  model_(model),
  vis_(vis)
{
}

double VeloToAsusCalibrator::eval(const VeloToAsusCalibration& cal) const
{
  SequenceAlignmentMDE mde(model_, frames_, pcds_);
  double rx, ry, rz, tx, ty, tz;
  generateXYZYPR(cal.veloToAsus(), rx, ry, rz, tx, ty, tz);

  ArrayXd x(7);
  x << cal.offset_, rx, ry, rz, tx, ty, tz;
  return mde.eval(x);
}

VeloToAsusCalibration VeloToAsusCalibrator::search(double* final_value) const
{
  cout << "Initializing search using PrimeSenseModel: " << endl;
  cout << model_.status("  ");
  SequenceAlignmentMDE::Ptr mde(new SequenceAlignmentMDE(model_, frames_, pcds_));
  GridSearch gs(7);
  gs.verbose_ = false;
  gs.view_handler_ = vis_;
  gs.objective_ = mde;
  //gs.num_scalings_ = 9;
  gs.num_scalings_ = 22;
  double max_res_time = 0.25;
  double max_res_rot = 2.5 * M_PI / 180.0;
  double max_res_trans = 0.1;
  gs.max_resolutions_ << max_res_time, max_res_rot, max_res_rot, max_res_rot, max_res_trans, max_res_trans, max_res_trans;
  int gr = 3;
  gs.grid_radii_ << gr, gr, gr, gr, gr, gr, gr;
  //double sf = 0.5;
  double sf = 0.75;
  gs.scale_factors_ << sf, sf, sf, sf, sf, sf, sf;
  gs.couplings_ << 4, 0, 1, 2, 1, 0, 3;  // Search over (pitch, y) and (yaw, x) jointly.
  
  ArrayXd x = gs.search(ArrayXd::Zero(7));
  cout << "GridSearch solution: " << x.transpose() << endl;
  if(final_value) {
    *final_value = mde->eval(x);
    cout << "Final objective function value: " << *final_value << endl;
  }
  
  VeloToAsusCalibration cal;
  cal.setVeloToAsus(generateTransform(x(1), x(2), x(3), x(4), x(5), x(6)));
  cal.offset_ = x(0);
  cout << "Final objective function value: " << eval(cal) << endl;
  
  return cal;
}
