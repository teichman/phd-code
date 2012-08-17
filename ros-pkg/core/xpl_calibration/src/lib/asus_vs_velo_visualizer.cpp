#include <xpl_calibration/asus_vs_velo_visualizer.h>

using namespace std;
using namespace Eigen;
using namespace rgbd;
namespace bfs = boost::filesystem;

VeloSequence::VeloSequence(std::string root_path) :
  root_path_(root_path)
{
  bfs::recursive_directory_iterator it(root_path_), eod;
  BOOST_FOREACH(const bfs::path& p, make_pair(it, eod)) {
    ROS_ASSERT(is_regular_file(p));
    if(bfs::extension(p).compare(".pcd") == 0)
      pcd_names_.push_back(p.leaf());
    else if(bfs::extension(p).compare(".clk") == 0)
      clk_names_.push_back(p.leaf());
  }
  sort(pcd_names_.begin(), pcd_names_.end());
  sort(clk_names_.begin(), clk_names_.end());

  // -- Load timestamps.
  timestamps_.resize(clk_names_.size());
  for(size_t i = 0; i < clk_names_.size(); ++i) {
    ifstream fs((root_path_ + "/" + clk_names_[i]).c_str());
    ROS_ASSERT(fs.is_open());
    fs >> timestamps_[i];
    fs.close();
  }
}

Cloud::Ptr VeloSequence::getCloud(size_t idx) const
{
  ROS_ASSERT(idx < size());
  
  Cloud::Ptr pcd(new Cloud);
  pcl::io::loadPCDFile<pcl::PointXYZRGB>(root_path_ + "/" + pcd_names_[idx], *pcd);
  pcd->header.stamp.fromSec(timestamps_[idx]);
  return pcd;
}

VeloToAsusCalibration::VeloToAsusCalibration() :
  offset_(0),
  velo_to_asus_(Affine3f::Identity())
{
}

AsusVsVeloVisualizer::AsusVsVeloVisualizer(rgbd::StreamSequence::ConstPtr sseq, VeloSequence::ConstPtr vseq) :
  skip_(20),
  num_pixel_plots_(20),
  sseq_(sseq),
  vseq_(vseq),
  vw_("Visualizer", 0.3),
  velo_idx_(0),
  asus_idx_(0),
  sseq_start_(0),
  velo_(new Cloud),
  asus_(new Cloud),
  vis_(new Cloud),
  unwarp_(true)
{
  sseq_start_ = sseq_->timestamps_[0];
  incrementVeloIdx(2);
  cal_.velo_to_asus_ = generateTransform(- M_PI / 2.0, 0, -M_PI / 2.0, 0, 0, 0).inverse();
  vw_.vis_.registerPointPickingCallback(&AsusVsVeloVisualizer::pointPickingCallback, *this);
  setColorScheme("monitor");
  
  mpliBegin();
  VectorXd tmp;
  ROS_ASSERT(tmp.rows() == 0);
}

void AsusVsVeloVisualizer::setColorScheme(std::string name)
{
  ROS_ASSERT(name == "publication" || name == "monitor");
  
  if(name == "publication") {
    vw_.vis_.setBackgroundColor(255, 255, 255);
  }
  else if(name == "monitor") {
    vw_.vis_.setBackgroundColor(0, 0, 0);
  }

  color_scheme_ = name;
}

void AsusVsVeloVisualizer::toggleColorScheme()
{
  if(color_scheme_ == "publication")
    setColorScheme("monitor");
  else if(color_scheme_ == "monitor")
    setColorScheme("publication");
}

void AsusVsVeloVisualizer::pointPickingCallback(const pcl::visualization::PointPickingEvent& event, void* cookie)
{
  if(event.getPointIndex() == -1)
    return;
    
  Point pt;
  event.getPoint(pt.x, pt.y, pt.z);
  cout << "Selected point: " << pt.x << ", " << pt.y << ", " << pt.z << endl;
  vw_.vis_.removeAllShapes();
  
  Point origin;
  origin.x = 0;
  origin.y = 0;
  origin.z = 0;
  if(color_scheme_ == "publication")
    vw_.vis_.addArrow<Point, Point>(origin, pt, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, "line");
  else {
    ROS_ASSERT(color_scheme_ == "monitor");
    vw_.vis_.addArrow<Point, Point>(origin, pt, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, "line");
  }
}

void AsusVsVeloVisualizer::colorPoint(rgbd::Point* pt) const
{
  double increment = 3;
  double thresh0 = -3;
  double thresh1 = thresh0 + increment;
  double thresh2 = thresh1 + increment;
  double thresh3 = thresh2 + increment;
  double height = pt->y;
  
  if(height < thresh0) {
    pt->b = 0;
    pt->g = 0;
    pt->r = 255;
  }
  else if(height >= thresh0 && height < thresh1) {
    int val = (height - thresh0) / (thresh1 - thresh0) * 255.;
    pt->b = val;
    pt->g = val;
    pt->r = 255 - val;
  }
  else if(height >= thresh1 && height < thresh2) {
    int val = (height - thresh1) / (thresh2 - thresh1) * 255.;
    pt->b = 255;
    pt->g = 255 - val;
    pt->r = 0;
  }
  else if(height >= thresh2 && height < thresh3) {
    int val = (height - thresh2) / (thresh3 - thresh2) * 255.;
    pt->b = 255 - val;
    pt->g = val;
    pt->r = 0;
  }
  else {
    pt->b = 0;
    pt->g = 255;
    pt->r = 0;
  }
}

inline VectorXd vectorize(const Eigen::MatrixXd& mat)
{
  VectorXd vec(mat.rows() * mat.cols());
  int idx = 0;
  for(int y = 0; y < mat.rows(); ++y)
    for(int x = 0; x < mat.cols(); ++x, ++idx)
      vec(idx) = mat(y, x);
  return vec;
}

void AsusVsVeloVisualizer::updateDisplay(int velo_idx, const Eigen::Affine3f& transform, double offset)
{
  // -- Get corresponding clouds.
  velo_ = filterVelo(vseq_->getCloud(velo_idx));
  double min_dt;
  asus_idx_ = findAsusIdx(velo_->header.stamp.toSec() + offset, &min_dt);
  asus_ = sseq_->getCloud(asus_idx_);
  if(!asus_)
    ROS_WARN_STREAM("Bad asus idx " << asus_idx_ << " / " << sseq_->size() << flush);
  //cout << asus_idx_ << ": " << asus_->size() << " " << asus_->width << " " << asus_->height << endl;

  // -- Draw.
  for(size_t i = 0; i < asus_->size(); ++i)
    colorPoint(&asus_->at(i));
  vw_.vis_.removeAllShapes();
  vis_->clear();
  pcl::transformPointCloud(*velo_, *vis_, transform);

  if(weights_.rows() != 0 && unwarp_) {
    Cloud unwarped(*asus_);
    ROS_ASSERT(unwarped.height == 480);
    VectorXd ms(4);
    VectorXd us(4);
    VectorXd vs(4);
    int idx = 0;
    for(size_t y = 0; y < unwarped.height; ++y) {
      for(size_t x = 0; x < unwarped.width; ++x, ++idx) {
	if(!isFinite(unwarped[idx]))
	  continue;
	double m = unwarped[idx].getVector3fMap().norm() / 10.0;
	ms << 1, m, m*m, m*m*m;
	double u = (double)x / 640.0;
	us << 1, u, u*u, u*u*u;
	double v = (double)y / 480.0;
	vs << 1, v, v*v, v*v*v;
	//VectorXd x = vectorize(vectorize(us * ms.transpose()) * vs.transpose());
	VectorXd x = vectorize(us * ms.transpose());
	double estimated_range = x.dot(weights_);
	Vector3f pt = unwarped[idx].getVector3fMap();
	pt = pt / pt.norm() * estimated_range;
	unwarped[idx].getVector3fMap() = pt;
      }
    }
    *vis_ += unwarped;
  }
  else
    *vis_ += *asus_;

  vw_.showCloud(vis_);
}
 
void AsusVsVeloVisualizer::run()
{
  while(true) {
    updateDisplay(velo_idx_, cal_.velo_to_asus_, cal_.offset_);

    char key = vw_.waitKey();
    switch(key) {
    case 27:
      return;
      break;
    case 'C':
      toggleColorScheme();
      break;
    case 'm':
      unwarp_ = !unwarp_;
      cout << "unwarp_: " << unwarp_ << endl;
      break;
    case 'p':
      play(false);
      break;
    case 'v':
      play(true);
      break;
    case 'V':
      visualizeDistortion();
      break;
    case 'A':
      accumulateStatistics();
      break;
    case 'M':
      fitModel();
      break;
    case 'c':
      calibrate();
      break;
    case 'P':
      generateHeatMap();
      break;
    case 'f':
      cout << "-- Frame stats --" << endl;
      cout << "Velo timestamp: " << setprecision(16) << velo_->header.stamp.toSec() << endl;
      cout << "Offset: " << cal_.offset_ << endl;
      cout << "Asus timestamp: " << sseq_->timestamps_[asus_idx_] - sseq_start_ << endl;
      cout << "velo + offset - asus: " << velo_->header.stamp.toSec() + cal_.offset_ - (sseq_->timestamps_[asus_idx_] - sseq_start_) << endl;
      cout << "Loss for this frame: " << getLossFunction()->eval(ArrayXd::Zero(1)) << endl;
      break;
    case 'S':
      saveAll("manual");
      break;
    case ',':
      incrementVeloIdx(-1);
      break;
    case '.':
      incrementVeloIdx(1);
      break;
    case '<':
      incrementVeloIdx(-10);
      break;
    case '>':
      incrementVeloIdx(10);
      break;
    case '@':
      incrementVeloIdx(-100);
      break;
    case '#':
      incrementVeloIdx(100);
      break;
    case '[':
      incrementOffset(-1.0 / 30.0);
      break;
    case ']':
      incrementOffset(1.0 / 30.0);
      break;
    case '{':
      incrementOffset(-10.0 / 30.0);
      break;
    case '}':
      incrementOffset(10.0 / 30.0);
      break;
    case 'x':
      cal_.velo_to_asus_ = generateTransform(1.0 * M_PI / 180.0, 0, 0, 0, 0, 0) * cal_.velo_to_asus_;
      break;
    case 'X':
      cal_.velo_to_asus_ = generateTransform(-1.0 * M_PI / 180.0, 0, 0, 0, 0, 0) * cal_.velo_to_asus_;
      break;
    case 'y':
      cal_.velo_to_asus_ = generateTransform(0, 1.0 * M_PI / 180.0, 0, 0, 0, 0) * cal_.velo_to_asus_;
      break;
    case 'Y':
      cal_.velo_to_asus_ = generateTransform(0, -1.0 * M_PI / 180.0, 0, 0, 0, 0) * cal_.velo_to_asus_;
      break;
    case 'z':
      cal_.velo_to_asus_ = generateTransform(0, 0, 1.0 * M_PI / 180.0, 0, 0, 0) * cal_.velo_to_asus_;
      break;
    case 'Z':
      cal_.velo_to_asus_ = generateTransform(0, 0, -1.0 * M_PI / 180.0, 0, 0, 0) * cal_.velo_to_asus_;
      break;
    case 'w':
      cal_.velo_to_asus_ = generateTransform(0, 0, 0, 0.1, 0, 0) * cal_.velo_to_asus_;
      break;
    case 's':
      cal_.velo_to_asus_ = generateTransform(0, 0, 0, -0.1, 0, 0) * cal_.velo_to_asus_;
      break;
    case 'a':
      cal_.velo_to_asus_ = generateTransform(0, 0, 0, 0, 0.1, 0) * cal_.velo_to_asus_;
      break;
    case 'd':
      cal_.velo_to_asus_ = generateTransform(0, 0, 0, 0, -0.1, 0) * cal_.velo_to_asus_;
      break;
    case 'E':
      cal_.velo_to_asus_ = generateTransform(0, 0, 0, 0, 0, 0.1) * cal_.velo_to_asus_;
      break;
    case 'e':
      cal_.velo_to_asus_ = generateTransform(0, 0, 0, 0, 0, -0.1) * cal_.velo_to_asus_;
      break;
    default:
      break;
    }
  }
}

void AsusVsVeloVisualizer::incrementVeloIdx(int val)
{
  velo_idx_ += val;
  if(velo_idx_ < 0)
    velo_idx_ = 0;
  if(velo_idx_ >= (int)vseq_->size())
    velo_idx_ = vseq_->size() - 1;
}

void AsusVsVeloVisualizer::incrementOffset(double dt)
{
  cal_.offset_ += dt;
  cout << "Using offset: " << cal_.offset_ << endl;
}

int AsusVsVeloVisualizer::findAsusIdx(double ts, double* dt_out) const
{
  int idx = -1;
  double min_dt = numeric_limits<double>::max();
  for(size_t i = 0; i < sseq_->timestamps_.size(); ++i) {
    double dt = fabs(ts - (sseq_->timestamps_[i] - sseq_start_));
    if(dt < min_dt) {
      min_dt = dt;
      idx = i;
    }
  }
  if(dt_out)
    *dt_out = min_dt;
  
  return idx;
}

rgbd::Cloud::Ptr AsusVsVeloVisualizer::filterAsus(rgbd::Cloud::ConstPtr asus) const
{
  Cloud::Ptr filtered(new Cloud(*asus));
  Point nopt;
  nopt.x = std::numeric_limits<float>::quiet_NaN();
  nopt.y = std::numeric_limits<float>::quiet_NaN();
  nopt.z = std::numeric_limits<float>::quiet_NaN();
  for(size_t i = 0; i < filtered->size(); ++i)
    if(isFinite(filtered->at(i)) && filtered->at(i).z > 8)
      filtered->at(i) = nopt;
  assert(filtered->isOrganized());
  return filtered;
}

rgbd::Cloud::Ptr AsusVsVeloVisualizer::filterVelo(rgbd::Cloud::ConstPtr velo) const
{
  double thresh = 30.0 * M_PI / 180.0;
  rgbd::Cloud::Ptr filtered(new Cloud);
  filtered->height = 1;
  assert(!filtered->isOrganized());
  
  filtered->reserve(velo->size() / 10.0);
  for(size_t i = 0; i < velo->size(); ++i) {
    const Point& pt = velo->at(i);
    if(!isFinite(pt))
      continue;
    double range = pt.getVector3fMap().norm();
    if(range > 15 || range < 0.5)
      continue;
    double yaw = atan2(pt.y, pt.x);
    if(fabs(yaw) > thresh)
      continue;

    filtered->push_back(pt);
  }

  filtered->header.stamp.fromSec(velo->header.stamp.toSec());
  return filtered;
}

LossFunction::Ptr AsusVsVeloVisualizer::getLossFunction() const
{
  // -- Set up loss function.
  pipeline::Params params;
  params.set<double>("TimeCorrespondenceThreshold", 0.1);
  params.set<double>("DistanceThreshold", 0.1);
  params.set<double>("Seq0Fx", 525);
  params.set<double>("Seq0Fy", 525);
  params.set<double>("Seq0Cx", 320);
  params.set<double>("Seq0Cy", 240);

  vector<Cloud::ConstPtr> pcds0;
  pcds0.push_back(filterAsus(asus_));
  vector<KdTree::Ptr> trees0;
  KdTree::Ptr tree(new KdTree);
  tree->setInputCloud(pcds0[0]);
  trees0.push_back(tree);

  vector<Cloud::Ptr> pcds1;
  Cloud::Ptr velo(new Cloud(*velo_));
  pcl::transformPointCloud(*velo, *velo, cal_.velo_to_asus_);
  velo->header.stamp.fromSec(asus_->header.stamp.toSec());  // Make sure LF compares the two.
  pcds1.push_back(velo);
  assert(!velo_->isOrganized());
  assert(!velo->isOrganized());
  
  LossFunction::Ptr lf(new LossFunction(trees0, pcds0, pcds1, params));
  return lf;
}

void AsusVsVeloVisualizer::calibrate()
{
  // -- Choose Velodyne keyframes.
  int num_keyframes = 25;
  int spacing = 30;
  int buffer = 50;
  vector<rgbd::Cloud::Ptr> velo_keyframes;
  int idx = buffer;
  while(true) {
    updateDisplay(idx, cal_.velo_to_asus_, cal_.offset_);
    
    velo_keyframes.push_back(filterVelo(vseq_->getCloud(idx)));
    pcl::transformPointCloud(*velo_keyframes.back(), *velo_keyframes.back(), cal_.velo_to_asus_);
    idx += spacing;
    if((int)velo_keyframes.size() >= num_keyframes)
      break;
    if(idx > ((int)vseq_->size() - buffer))
      break;
  }
  cout << "Loaded " << velo_keyframes.size() << " velodyne keyframes." << endl;
  updateDisplay(velo_idx_, cal_.velo_to_asus_, cal_.offset_);
  
  // -- Load Asus frames in the vicinity of the Velodyne keyframes.
  vector<Cloud::ConstPtr> pcds0;
  vector<KdTree::Ptr> trees0;
  int window = 15;
  for(size_t i = 0; i < velo_keyframes.size(); ++i) {
    int idx = findAsusIdx(velo_keyframes[i]->header.stamp.toSec() + cal_.offset_);
    for(int j = max(0, idx - window); j <= min(idx + window, (int)sseq_->size()); ++j) {
      Cloud::Ptr asus = sseq_->getCloud(j);
      double ts = asus->header.stamp.toSec() - sseq_start_;
      asus->header.stamp.fromSec(ts);
      pcds0.push_back(filterAsus(asus));
      
      KdTree::Ptr tree(new KdTree);
      tree->setInputCloud(pcds0.back());
      trees0.push_back(tree);
    }
    cout << "Loaded nearby asus frames for velo keyframe " << i << endl;
  }

  // -- Set up loss function.
  pipeline::Params params;
  params.set<double>("TimeCorrespondenceThreshold", 0.1);
  params.set<double>("DistanceThreshold", 0.1);
  params.set<double>("Seq0Fx", 525);
  params.set<double>("Seq0Fy", 525);
  params.set<double>("Seq0Cx", 320);
  params.set<double>("Seq0Cy", 240);
  LossFunction::Ptr lf(new LossFunction(trees0, pcds0, velo_keyframes, params));
  
  int iter = 0;
  while(true) {
    cout << "============================================================" << endl;
    cout << "Iteration " << iter << endl;
    cout << "============================================================" << endl;
    ++iter;
            
    // -- Search over transform incremental update.
    Affine3f incremental_transform = gridSearchTransform(lf);
    cout << "Found transform " << endl << incremental_transform.matrix() << endl;
    cal_.velo_to_asus_ = incremental_transform * cal_.velo_to_asus_;
    // Apply the incremental transform.
    for(size_t i = 0; i < velo_keyframes.size(); ++i)
      pcl::transformPointCloud(*velo_keyframes[i], *velo_keyframes[i], incremental_transform);

    // -- Search over sync offset incremental update.
    double dt = gridSearchSync(lf);
    cout << "Found dt = " << dt << endl;
    cal_.offset_ += dt;
    // Apply the update.
    for(size_t i = 0; i < velo_keyframes.size(); ++i) {
      double ts = velo_keyframes[i]->header.stamp.toSec() + dt;
      velo_keyframes[i]->header.stamp.fromSec(ts);
    }
    
    // -- Determine if we're done.
    cout << incremental_transform.matrix() << endl;
    double fro = (incremental_transform.matrix() - Affine3f::Identity().matrix()).norm();
    cout << "frobenius norm of (T - I): " << fro << endl;
    if(fro < 1e-3 && dt < 0.015)
      break;
  }
  
  cout << "Done calibrating." << endl;
  cout << "Final sync offset: " << cal_.offset_ << endl;
  cout << "Final transform: " << cal_.velo_to_asus_.matrix() << endl;
  cal_.save("calibration-autosave");
}

Eigen::Affine3f AsusVsVeloVisualizer::gridSearchTransform(ScalarFunction::Ptr lf)
{
  cout << "Starting grid search over transform." << endl;
  GridSearch gs(6);
  gs.verbose_ = false;
  gs.view_handler_ = this;
  gs.objective_ = lf;
  gs.num_scalings_ = 5;
  double maxrr = 2.0 * M_PI / 180.0;
  double maxrt = 0.2;
  gs.max_resolutions_ << maxrr, maxrr, maxrr, maxrt, maxrt, maxrt;
  int gr = 3;
  gs.grid_radii_ << gr, gr, gr, gr, gr, gr;
  double sf = 0.5;
  gs.scale_factors_ << sf, sf, sf, sf, sf, sf;
  gs.couplings_ << 0, 1, 2, 1, 0, 3;  // Search over (pitch, y) and (yaw, x) jointly.
  
  ArrayXd x = gs.search(ArrayXd::Zero(6));
  cout << "GridSearch solution: " << x.transpose() << endl;
  return generateTransform(x(0), x(1), x(2), x(3), x(4), x(5));
}

double AsusVsVeloVisualizer::gridSearchSync(ScalarFunction::Ptr lf)
{
  GridSearch gs(1);
  gs.verbose_ = false;
  gs.view_handler_ = this;
  gs.objective_ = lf;
  gs.max_resolutions_ << 0.25;
  gs.grid_radii_ << 2;
  gs.scale_factors_ << 0.5;
  gs.num_scalings_ = 4;

  cout << "Starting grid search over offset." << endl;
  ArrayXd x = gs.search(ArrayXd::Zero(1));
  return x(0);
}

void AsusVsVeloVisualizer::align()
{
  double before = getLossFunction()->eval(ArrayXd::Zero(1));
  cal_.velo_to_asus_ = gridSearchTransform(getLossFunction()) * cal_.velo_to_asus_;

  cout << "Loss before alignment: " << before << endl;
  cout << "Loss after alignment: " << getLossFunction()->eval(ArrayXd::Zero(1)) << endl;
  cout << "Loss after alignment: " << getLossFunction()->eval(ArrayXd::Zero(6)) << endl;
}

void AsusVsVeloVisualizer::handleGridSearchUpdate(const Eigen::ArrayXd& x, double objective)
{
  cout << "Grid search improvement.  New objective: " << objective << endl;
  Affine3f incremental_transform = Affine3f::Identity();
  double dt = 0;
  if(x.rows() == 6) { 
    incremental_transform = generateTransform(x(0), x(1), x(2), x(3), x(4), x(5));
  }
  else if(x.rows() == 1) { 
    dt = x(0);
    cout << "offset: " << cal_.offset_ + dt << endl;
  }
  
  updateDisplay(velo_idx_, incremental_transform * cal_.velo_to_asus_, cal_.offset_ + dt);
  static int num = 0;
  ostringstream oss;
  oss << "gridsearch" << setw(5) << setfill('0') << num << ".png";
  vw_.vis_.saveScreenshot(oss.str());
  ++num;
}

void AsusVsVeloVisualizer::play(bool save)
{
  for(; velo_idx_ < (int)vseq_->size(); ++velo_idx_) {
    updateDisplay(velo_idx_, cal_.velo_to_asus_, cal_.offset_);
    char key = vw_.waitKey(20);
    if(key != 0)
      break;
    
    if(save) { 
      ostringstream oss;
      oss << "video" << setw(5) << setfill('0') << velo_idx_ << ".png";
      vw_.vis_.saveScreenshot(oss.str());
    }
  }
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
  velo_to_asus_ = mat;
}

void AsusVsVeloVisualizer::accumulateStatistics()
{
  updateDisplay(20, cal_.velo_to_asus_, cal_.offset_);
  //cout << asus_->size() << " " << asus_->width << " " << asus_->height << endl;
  ROS_ASSERT(asus_);
  ROS_ASSERT(asus_->height > 0 && asus_->width > 0);
  Projector proj;
  proj.fx_ = 525;
  proj.fy_ = 525;
  proj.cx_ = asus_->width / 2;
  proj.cy_ = asus_->height / 2;
  proj.height_ = asus_->height;
  proj.width_ = asus_->width;

  // -- Accumulate statistics.
  statistics_.clear();
  statistics_.resize(proj.height_, vector<PixelStats>(proj.width_));
  for(size_t i = 0; i < statistics_.size(); ++i)
    for(size_t j = 0; j < statistics_[i].size(); ++j)
      statistics_[i][j].reserve(ceil((double)sseq_->size() / (double)skip_));
  
  double min_mult = 0.85;
  double max_mult = 1.25;
  Cloud::Ptr transformed(new Cloud);
  for(size_t i = 20; i < vseq_->size(); i += skip_) {
    updateDisplay(i, cal_.velo_to_asus_, cal_.offset_);
    assert(asus_->isOrganized());
    pcl::transformPointCloud(*velo_, *transformed, cal_.velo_to_asus_);
    for(size_t j = 0; j < transformed->size(); ++j) {
      if(!isFinite(transformed->at(j)))
	continue;

      ProjectedPoint pp;
      proj.project(transformed->at(j), &pp);
      if(pp.u_ < 0 || pp.u_ >= proj.width_ || pp.v_ < 0 || pp.v_ >= proj.height_)
	continue;
		   
      Point asuspt = asus_->at(pp.u_ + pp.v_ * proj.width_);
      if(!isFinite(asuspt))
	continue;

      // If the range is completely off, assume it's due to misalignment and not distortion.
      double measurement = asuspt.getVector3fMap().norm();
      double range = transformed->at(j).getVector3fMap().norm();
      double mult = range / measurement;
      if(mult < min_mult || mult > max_mult)
	continue;

      if(isinf(measurement) || isnan(measurement) || isinf(range) || isnan(range))
	continue;
      statistics_[pp.v_][pp.u_].addPoint(range, measurement);
    }
  }
}

void AsusVsVeloVisualizer::fitModel()
{
  if(statistics_.empty()) {
    cout << "You must accumulate statistics first." << endl;
    return;
  }
  cout << "Fitting model." << endl;

  int num_tr_ex = 0;
  for(size_t y = 0; y < statistics_.size(); ++y)
    for(size_t x = 0; x < statistics_[y].size(); ++x)
      for(size_t i = 0; i < statistics_[y][x].asus_.size(); ++i)
	++num_tr_ex;
  cout << num_tr_ex << " training examples." << endl;
  
  //int num_features = 4 * 4 * 4;
  int num_features = 4 * 4;
  MatrixXd X(num_features, num_tr_ex);
  VectorXd Y(num_tr_ex);
  int idx = 0;
  VectorXd us(4);
  VectorXd vs(4);
  VectorXd ms(4);
  VectorXd measurements(num_tr_ex);
  for(size_t y = 0; y < statistics_.size(); ++y) {
    for(size_t x = 0; x < statistics_[y].size(); ++x) {
      for(size_t i = 0; i < statistics_[y][x].asus_.size(); ++i, ++idx) { 
	double u = (double)x / 640.0;
	us << 1, u, u*u, u*u*u;
	double m = statistics_[y][x].asus_[i] / 10.0;
	ms << 1, m, m*m, m*m*m;
	double v = (double)y / 480.0;
	vs << 1, v, v*v, v*v*v;
	//X.col(idx) = vectorize(vectorize(us * ms.transpose()) * vs.transpose());
	X.col(idx) = vectorize(us * ms.transpose());
	Y(idx) = statistics_[y][x].velo_[i];
	measurements(idx) = statistics_[y][x].asus_[i];
      }
    }
  }

  MatrixXd xxt = X * X.transpose();
  ROS_ASSERT(xxt.rows() == 4*4);
  //ROS_ASSERT(xxt.rows() == 4*4*4);
  VectorXd b = X*Y;
  weights_ = xxt.ldlt().solve(b);
  cout << "Weights: " << weights_.transpose() << endl;
  
  VectorXd pre_differences = measurements - Y;
  double pre_obj = pre_differences.array().pow(2).sum() / (double)Y.rows();
  cout << "Mean error before fitting model: " << pre_obj << endl;

  VectorXd differences = (weights_.transpose() * X).transpose() - Y;
  double obj = differences.array().pow(2).sum() / (double)Y.rows();
  cout << "Mean error after fitting model: " << obj << endl;
}


void AsusVsVeloVisualizer::saveExtrinsics(std::string tag) const
{
  cal_.save("extrinsics" + tag);
  cout << "Saved calibration to \"" << "extrinsics" << tag << "\"" << endl;
}

void AsusVsVeloVisualizer::saveDistortionModel(std::string tag) const
{
  eigen_extensions::saveASCII(weights_, "depth_distortion_model" + tag + ".eig.txt");
  cout << "Saved depth distortion model to \"" << "depth_distortion_model" + tag + ".eig.txt\"" << endl;
}

void AsusVsVeloVisualizer::saveAll(std::string tag) const
{
  saveExtrinsics(tag);
  saveDistortionModel(tag);
}

void AsusVsVeloVisualizer::visualizeDistortion()
{
  if(statistics_.empty()) {
    cout << "You must accumulate statistics first." << endl;
    return;
  }
  cout << "Visualizing distortion." << endl;
  
  // -- Generate a heat map.
  int width = asus_->width;
  int height = asus_->height;
  Eigen::MatrixXd mean(height, width);
  Eigen::MatrixXd stdev(height, width);
  Eigen::MatrixXd counts(height, width);
  mean.setZero();
  stdev.setZero();
  counts.setZero();
  for(int y = 0; y < mean.rows(); ++y) 
    for(int x = 0; x < mean.cols(); ++x) 
      statistics_[y][x].stats(&mean.coeffRef(y, x), &stdev.coeffRef(y, x), &counts.coeffRef(y, x));
  
  mpliExport(mean);
  mpliExport(stdev);
  mpliExport(counts);
  mpliPrintSize();
  mpliExecuteFile(ros::package::getPath("xpl_calibration") + "/plot_multipliers_image.py");

  // -- Range vs u image.
  {
    double max_range = 13;
    int num_range_bins = 500;
    double bin_size = max_range / (double)num_range_bins;
    
    Eigen::ArrayXXd mean(num_range_bins, width);
    Eigen::ArrayXXd counts(num_range_bins, width);
    mean.setZero();
    counts.setZero();
    for(size_t y = 0; y < statistics_.size(); ++y)  {
      ROS_ASSERT((int)statistics_[y].size() == width);
      for(size_t x = 0; x < statistics_[y].size(); ++x) {
	for(size_t i = 0; i < statistics_[y][x].asus_.size(); ++i) { 
	  double range = statistics_[y][x].velo_[i];
	  int range_bin = (max_range - range) / bin_size;
	  if(range_bin >= num_range_bins || range_bin < 0)
	    continue;
	  
	  double mult = range / statistics_[y][x].asus_[i];
	  counts(range_bin, x) += 1;
	  mean(range_bin, x) += mult;
	}
      }
    }
    mean /= counts;
    for(int y = 0; y < mean.rows(); ++y)
      for(int x = 0; x < mean.cols(); ++x)
	if(isnan(mean(y, x)))
	  mean(y, x) = 1;
        
    Eigen::ArrayXXd stdev(num_range_bins, width);
    stdev.setZero();
    for(size_t y = 0; y < statistics_.size(); ++y)  { 
      for(size_t x = 0; x < statistics_[y].size(); ++x) {
	for(size_t i = 0; i < statistics_[y][x].asus_.size(); ++i) { 
	  double range = statistics_[y][x].velo_[i];
	  int range_bin = (max_range - range) / bin_size;
	  if(range_bin >= num_range_bins || range_bin < 0)
	    continue;
	  
	  double mult = range / statistics_[y][x].asus_[i];
	  stdev(range_bin, x) += pow(mult - mean(range_bin, x), 2);
	}
      }
    }
    stdev = (stdev / counts).sqrt();
    for(int y = 0; y < stdev.rows(); ++y)
      for(int x = 0; x < stdev.cols(); ++x)
	if(isnan(stdev(y, x)))
	  stdev(y, x) = 0;

    mpliExport(max_range);
    mpliExport(bin_size);
    mpliNamedExport("mean", mean);
    mpliNamedExport("stdev", stdev);
    mpliNamedExport("counts", counts);
    mpliPrintSize();
    mpliExecuteFile(ros::package::getPath("xpl_calibration") + "/plot_u_range_multipliers.py");
  }
  
  // -- For some random pixels, generate a scatter plot of asus range vs velo range.
  srand(0);  // All runs should produce plots for the same set of pixels.
  for(int i = 0; i < num_pixel_plots_; ++i) {
    int u = rand() % width;
    int v = rand() % height;
    const PixelStats& ps = statistics_[v][u];
    if(!ps.valid())
      continue;

    cout << "Plotting distortion for pixel " << u << " " << v << endl;
    mpliNamedExport("velo", ps.velo_);
    mpliNamedExport("asus", ps.asus_);
    mpliNamedExport<int>("width", width);
    mpliNamedExport<int>("height", height);
    mpliExport(u);
    mpliExport(v);
    mpliExecuteFile(ros::package::getPath("xpl_calibration") + "/plot_beam_scatter.py");
  }
}

void AsusVsVeloVisualizer::generateHeatMap()
{
  vector<double> radii;
  vector<double> multipliers;
  radii.reserve(1e8);
  multipliers.reserve(1e8);

  Projector proj;
  proj.fx_ = 525;
  proj.fy_ = 525;
  proj.cx_ = asus_->width / 2;
  proj.cy_ = asus_->height / 2;
  proj.height_ = asus_->height;
  proj.width_ = asus_->width;
  
  double min_mult = 0.85;
  double max_mult = 1.15;
  Cloud::Ptr transformed(new Cloud);
  for(size_t i = 20; i < vseq_->size(); i += skip_) {
    updateDisplay(i, cal_.velo_to_asus_, cal_.offset_);
    assert(asus_->isOrganized());
    pcl::transformPointCloud(*velo_, *transformed, cal_.velo_to_asus_);
    for(size_t j = 0; j < transformed->size(); ++j) {
      if(!isFinite(transformed->at(j)))
	continue;

      ProjectedPoint pp;
      proj.project(transformed->at(j), &pp);
      Point asuspt = asus_->at(pp.u_ + pp.v_ * proj.width_);
      if(!isFinite(asuspt))
	continue;

      // Only look at far-away points.
      // if(transformed->at(j).z < 6)
      // 	continue;

      // Only look at near points.
      if(transformed->at(j).z > 4)
      	continue;
      
      double mult = transformed->at(j).z / asuspt.z;
      // If the range is completely off, assume it's due to misalignment and not distortion.
      if(mult < min_mult || mult > max_mult)
	continue;
      
      radii.push_back(sqrt((double)pow(pp.u_ - proj.cx_, 2) + (double)pow(pp.v_ - proj.cy_, 2)) / 400.0);
      multipliers.push_back(mult);
    }
  }
  cout << "Got " << radii.size() << " points for the range plot." << endl;

  int num_bins = 100;
  Eigen::MatrixXd hist(num_bins, num_bins);
  hist.setZero();
  for(size_t i = 0; i < radii.size(); ++i) {
    int ridx = radii[i] * num_bins;
    if(ridx < 0 || ridx >= num_bins)
      continue;
    int midx = (1.0 - (multipliers[i] - min_mult) / (max_mult - min_mult)) * num_bins;
    if(midx < 0 || midx >= num_bins)
      continue;

    ++hist(midx, ridx);
  }
  for(int i = 0; i < hist.cols(); ++i)
    if(hist.col(i).sum() > 0)
      hist.col(i).normalize();

  mpliExport(num_bins);
  mpliExport(max_mult);
  mpliExport(min_mult);
  mpliExport(hist);
  mpliExecuteFile(ros::package::getPath("xpl_calibration") + "/plot_multipliers.py");
}

void PixelStats::addPoint(double velo, double asus)
{
  velo_.push_back(velo);
  asus_.push_back(asus);
}

void PixelStats::stats(double* mean, double* stdev, double* num) const
{
  *num = velo_.size();
  
  if(!valid()) {
    *mean = 1;
    *stdev = 0;
    return;
  }
  
  *mean = 0;
  for(size_t i = 0; i < velo_.size(); ++i)
    *mean += velo_[i] / asus_[i];
  *mean /= (double)velo_.size();
  
  *stdev = 0;
  for(size_t i = 0; i < velo_.size(); ++i)
    *stdev += pow(velo_[i] / asus_[i] - *mean, 2);
  *stdev /= (double)velo_.size();
  *stdev = sqrt(*stdev);
}

bool PixelStats::valid() const
{

  return velo_.size() > 5;
}
