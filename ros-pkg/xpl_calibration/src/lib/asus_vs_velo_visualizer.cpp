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
  sseq_(sseq),
  vseq_(vseq),
  velo_idx_(0),
  asus_idx_(0),
  sseq_start_(0),
  velo_(new Cloud),
  asus_(new Cloud),
  vis_(new Cloud)
{
  sseq_start_ = sseq_->timestamps_[0];
  incrementVeloIdx(2);
  cal_.velo_to_asus_ = generateTransform(- M_PI / 2.0, 0, -M_PI / 2.0, 0, 0, 0).inverse();
  vw_.vis_.registerPointPickingCallback(&AsusVsVeloVisualizer::pointPickingCallback, *this);

  mpliBegin();
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
  vw_.vis_.addArrow<Point, Point>(origin, pt, 0.0, 0.0, 1.0, "line");
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

void AsusVsVeloVisualizer::updateDisplay(int velo_idx, const Eigen::Affine3f& transform, double offset)
{
  // -- Get corresponding clouds.
  velo_ = filterVelo(vseq_->getCloud(velo_idx));
  double min_dt;
  asus_idx_ = findAsusIdx(velo_->header.stamp.toSec() + offset, &min_dt);
  asus_ = sseq_->getCloud(asus_idx_);

  // -- Draw.
  for(size_t i = 0; i < asus_->size(); ++i)
    colorPoint(&asus_->at(i));
  vw_.vis_.removeAllShapes();
  vis_->clear();
  pcl::transformPointCloud(*velo_, *vis_, transform);
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
    case 'p':
      play(false);
      break;
    case 'v':
      play(true);
      break;
    case 'A':
      align();
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
      cal_.save("calibration");
      cout << "Saved calibration to \"calibration\"" << endl;
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
  int spacing = 50;
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
  
  int skip = 10;
  double min_mult = 0.85;
  double max_mult = 1.15;
  Cloud::Ptr transformed(new Cloud);
  for(size_t i = 20; i < vseq_->size(); i += skip) {
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
  mpliExecuteFile("plot_multipliers.py");
}
