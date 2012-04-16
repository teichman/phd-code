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


AsusVsVeloVisualizer::AsusVsVeloVisualizer(rgbd::StreamSequence::ConstPtr sseq, VeloSequence::ConstPtr vseq) :
  sseq_(sseq),
  vseq_(vseq),
  velo_idx_(0),
  asus_idx_(0),
  offset_(0),
  sseq_start_(0),
  velo_(new Cloud),
  asus_(new Cloud),
  vis_(new Cloud),
  asus_to_velo_(Affine3f::Identity())
{
  sseq_start_ = sseq_->timestamps_[0];
  incrementVeloIdx(2);

  asus_to_velo_ = generateTransform(- M_PI / 2.0, 0, -M_PI / 2.0, 0, 0, 0);

  vw_.vis_.registerPointPickingCallback(&AsusVsVeloVisualizer::pointPickingCallback, *this);
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

void AsusVsVeloVisualizer::updateDisplay()
{
  vw_.vis_.removeAllShapes();
  vis_->clear();
  pcl::transformPointCloud(*asus_, *vis_, asus_to_velo_);
  for(size_t i = 0; i < vis_->size(); ++i)
    vis_->at(i).r = 255;

  *vis_ += *filter(velo_);
  vw_.showCloud(vis_);
}
 
void AsusVsVeloVisualizer::run()
{
  while(true) {
    sync();

    updateDisplay();

    char key = vw_.waitKey();
    switch(key) {
    case 27:
      return;
      break;
    case 'A':
      align();
      break;
    case 'C':
      calibrate();
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
      asus_to_velo_ = generateTransform(1.0 * M_PI / 180.0, 0, 0, 0, 0, 0) * asus_to_velo_;
      break;
    case 'X':
      asus_to_velo_ = generateTransform(-1.0 * M_PI / 180.0, 0, 0, 0, 0, 0) * asus_to_velo_;
      break;
    case 'y':
      asus_to_velo_ = generateTransform(0, 1.0 * M_PI / 180.0, 0, 0, 0, 0) * asus_to_velo_;
      break;
    case 'Y':
      asus_to_velo_ = generateTransform(0, -1.0 * M_PI / 180.0, 0, 0, 0, 0) * asus_to_velo_;
      break;
    case 'z':
      asus_to_velo_ = generateTransform(0, 0, 1.0 * M_PI / 180.0, 0, 0, 0) * asus_to_velo_;
      break;
    case 'Z':
      asus_to_velo_ = generateTransform(0, 0, -1.0 * M_PI / 180.0, 0, 0, 0) * asus_to_velo_;
      break;
    case 'w':
      asus_to_velo_ = generateTransform(0, 0, 0, 0.1, 0, 0) * asus_to_velo_;
      break;
    case 's':
      asus_to_velo_ = generateTransform(0, 0, 0, -0.1, 0, 0) * asus_to_velo_;
      break;
    case 'a':
      asus_to_velo_ = generateTransform(0, 0, 0, 0, 0.1, 0) * asus_to_velo_;
      break;
    case 'd':
      asus_to_velo_ = generateTransform(0, 0, 0, 0, -0.1, 0) * asus_to_velo_;
      break;
    case 'E':
      asus_to_velo_ = generateTransform(0, 0, 0, 0, 0, 0.1) * asus_to_velo_;
      break;
    case 'e':
      asus_to_velo_ = generateTransform(0, 0, 0, 0, 0, -0.1) * asus_to_velo_;
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

  sync();
}

void AsusVsVeloVisualizer::incrementOffset(double dt)
{
  offset_ += dt;
  cout << "Using offset: " << offset_ << endl;
  sync();
}

int AsusVsVeloVisualizer::findAsusIdx(double ts, double* dt_out) const
{
  int idx = -1;
  double min_dt = numeric_limits<double>::max();
  for(size_t i = 0; i < sseq_->timestamps_.size(); ++i) {
    double dt = fabs(ts - offset_ - (sseq_->timestamps_[i] - sseq_start_));
    if(dt < min_dt) {
      min_dt = dt;
      idx = i;
    }
  }
  if(dt_out)
    *dt_out = min_dt;
  
  return idx;
}

void AsusVsVeloVisualizer::sync()
{
  double min_dt;
  asus_idx_ = findAsusIdx(velo_->header.stamp.toSec(), &min_dt);
  
  cout << setprecision(16) << velo_->header.stamp.toSec() << " " << min_dt << " " << sseq_->timestamps_[asus_idx_] - sseq_start_ << endl;
  velo_ = vseq_->getCloud(velo_idx_);
  asus_ = sseq_->getCloud(asus_idx_);
  
  cout << "Loss: " << getLossFunction()->eval(ArrayXd::Zero(1)) << endl;
}

rgbd::Cloud::Ptr AsusVsVeloVisualizer::filter(rgbd::Cloud::ConstPtr velo) const
{
  double thresh = 30.0 * M_PI / 180.0;
  rgbd::Cloud::Ptr filtered(new Cloud);
  filtered->reserve(velo_->size() / 10.0);
  for(size_t i = 0; i < velo_->size(); ++i) {
    const Point& pt = velo_->at(i);
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
  Cloud::Ptr transformed(new Cloud);
  pcl::transformPointCloud(*asus_, *transformed, asus_to_velo_);
  transformed->header.stamp.fromSec(velo_->header.stamp.toSec()); // Make sure LF compares the two.
  pcds0.push_back(transformed);
  vector<KdTree::Ptr> trees0;
  KdTree::Ptr tree(new KdTree);
  tree->setInputCloud(pcds0[0]);
  trees0.push_back(tree);

  vector<Cloud::Ptr> pcds1;
  pcds1.push_back(filter(velo_));
  LossFunction::Ptr lf(new LossFunction(trees0, pcds0, pcds1, params));
  
  return lf;
}

void AsusVsVeloVisualizer::calibrate()
{
  // -- Choose Velodyne keyframes.
  int num_keyframes = 5;
  int spacing = 300;
  int buffer = 50;
  vector<rgbd::Cloud::Ptr> velo_keyframes;
  int idx = buffer;
  while(true) {
    velo_keyframes.push_back(filter(vseq_->getCloud(idx)));
    idx += spacing;
    if((int)velo_keyframes.size() >= num_keyframes)
      break;
    if(idx > ((int)vseq_->size() - buffer))
      break;
  }

  while(true) {
    // -- Load Asus frames in the vicinity of the Velodyne keyframes.
    //    Uses offset_ to find which frames to load, and applies offset_ to those frames.
    vector<Cloud::ConstPtr> pcds0;
    vector<KdTree::Ptr> trees0;
    int window = 30;  // ~one second in either direction. 
    for(size_t i = 0; i < velo_keyframes.size(); ++i) { 
      int idx = findAsusIdx(velo_keyframes[i]->header.stamp.toSec());
      for(int j = max(0, idx - window); j <= min(idx + window, (int)sseq_->size()); ++j) {
	Cloud::Ptr asus = sseq_->getCloud(j);
	Cloud::Ptr transformed(new Cloud);
	pcl::transformPointCloud(*asus, *transformed, asus_to_velo_);
	transformed->header.stamp.fromSec(asus->header.stamp.toSec() - sseq_start_);
	pcds0.push_back(transformed);
	
	KdTree::Ptr tree(new KdTree);
	tree->setInputCloud(pcds0.back());
	trees0.push_back(tree);
      }
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
    
    // -- Search over sync offset incremental update.
    double dt = gridSearchSync(lf);
    cout << "Found dt = " << dt << endl;
    // LossFunction adds dt to velo_keyframes timestamps.
    // Here, we want to subtract it.
    offset_ -= dt;
    // Apply the offset to the asus data before running grid search over transforms.
    for(size_t i = 0; i < pcds0.size(); ++i) {
      Cloud::Ptr updated(new Cloud(*pcds0[i]));
      double ts = pcds0[i]->header.stamp.toSec() + offset_;
      updated->header.stamp.fromSec(ts);
      pcds0[i] = updated;
    }
    
    // -- Search over transform incremental update.
    Affine3f incremental_transform = gridSearchTransform(lf);
    cout << "Found transform " << endl << incremental_transform.matrix() << endl;
    asus_to_velo_ = incremental_transform * asus_to_velo_;

    // -- Update display.
    sync();
    updateDisplay();
  }
}

Eigen::Affine3f AsusVsVeloVisualizer::gridSearchTransform(ScalarFunction::Ptr lf) const
{
  cout << "Starting grid search over transform." << endl;
  GridSearch gs(6);
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
  return generateTransform(x(0), x(1), x(2), x(3), x(4), x(5)).inverse();
}

double AsusVsVeloVisualizer::gridSearchSync(ScalarFunction::Ptr lf) const
{
  GridSearch gs(1);
  gs.objective_ = lf;
  gs.max_resolutions_ << 0.5;
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
  asus_to_velo_ = gridSearchTransform(getLossFunction()) * asus_to_velo_;

  cout << "Loss before alignment: " << before << endl;
  cout << "Loss after alignment: " << getLossFunction()->eval(ArrayXd::Zero(1)) << endl;
  cout << "Loss after alignment: " << getLossFunction()->eval(ArrayXd::Zero(6)) << endl;
}
