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

VeloToAsusCalibration VeloToAsusCalibrator::search() const
{
  cout << "Initializing search using PrimeSenseModel: " << endl;
  cout << model_.status("  ");
  MeanDepthError::Ptr mde(new MeanDepthError(model_, frames_, pcds_));
  GridSearch gs(7);
  gs.verbose_ = false;
  gs.view_handler_ = vis_;
  gs.objective_ = mde;
  gs.num_scalings_ = 7;
  double max_res_time = 0.25;
  double max_res_rot = 5.0 * M_PI / 180.0;
  double max_res_trans = 0.5;
  gs.max_resolutions_ << max_res_time, max_res_rot, max_res_rot, max_res_rot, max_res_trans, max_res_trans, max_res_trans;
  int gr = 3;
  gs.grid_radii_ << gr, gr, gr, gr, gr, gr, gr;
  double sf = 0.5;
  gs.scale_factors_ << sf, sf, sf, sf, sf, sf, sf;
  gs.couplings_ << 4, 0, 1, 2, 1, 0, 3;  // Search over (pitch, y) and (yaw, x) jointly.
  
  ArrayXd x = gs.search(ArrayXd::Zero(7));
  cout << "GridSearch solution: " << x.transpose() << endl;

  VeloToAsusCalibration cal;
  cal.setVeloToAsus(generateTransform(x(1), x(2), x(3), x(4), x(5), x(6)));
  cal.offset_ = x(0);
  return cal;
}
