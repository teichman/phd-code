#include <rgbd_sequence/tube_measurer.h>

using namespace std;
using namespace Eigen;

namespace rgbd
{

  void AcceptedPoints::serialize(std::ostream& out) const
  {
    out << size() << endl;
    for(size_t i = 0; i < size(); ++i) {
      at(i).first.serialize(out);
      at(i).second.serialize(out);
    }
  }

  void AcceptedPoints::deserialize(std::istream& in)
  {
    size_t sz;
    in >> sz;
    resize(sz);
    for(size_t i = 0; i < sz; ++i) { 
      in >> at(i).first;
      in >> at(i).second;
    }
  }

  TubeMeasurer::TubeMeasurer() :
    idx_(0),
    warped_(new Cloud)
  {
    vw_.vis_.registerPointPickingCallback(&TubeMeasurer::pointPickingCallback, *this);
  }
  
  void TubeMeasurer::run(const std::string& path, double actual_distance)
  {
    path_ = path;
    actual_distance_ = actual_distance;
    
    sseq_.load(path);
    proj_.fx_ = sseq_.fx_;
    proj_.fy_ = sseq_.fy_;
    proj_.cx_ = sseq_.cx_;
    proj_.cy_ = sseq_.cy_;
    Cloud::Ptr pcd = sseq_.getCloud(0);
    proj_.width_ = pcd->width;
    proj_.height_ = pcd->height;
    
    while(true) {
      Cloud::Ptr pcd = sseq_.getCloud(idx_);
      proj_.cloudToFrame(*pcd, &frame_);
      proj_.frameToCloud(frame_, warped_.get());
      
      vw_.showCloud(warped_);
      char key = vw_.waitKey();

      switch(key) {
	// Apparent 'q' doesn't get passed through, so ESC quits.
      case 27:
	return;
	break;
      case 'g':
	gridSearch();
	break;
      case 's':
	saveAccepted();
	break;
      case 'c':
	clearSelection();
	break;
      case 'l':
	loadAccepted();
	break;
      case 'd':
	cout << "Accepted points: " << endl;
	cout << accepted_;
	break;
      case 'a':
	acceptVisible();
	break;
      case ',':
	increment(-1);
	break;
      case '.':
	increment(1);
	break;
      case '{':
	incrementIntrinsics(-10, -10, 0, 0);
	break;
      case '}':
	incrementIntrinsics(10, 10, 0, 0);
	break;
      case '[':
	incrementIntrinsics(0, 0, -10, 0);
	break;
      case ']':
	incrementIntrinsics(0, 0, 10, 0);
	break;
      case 't':
	findTube(*warped_);
	break;
      default:
	break;
      }
    }
  }
  
  void TubeMeasurer::findTube(const rgbd::Cloud& pcd)
  {
    Cloud::Ptr brown(new Cloud);
    brown->reserve(pcd.size());
    Point br;
    br.r = 165;
    br.g = 155;
    br.b = 105;
    int thresh = 20;
    
    for(size_t i = 0; i < pcd.size(); ++i) {
      if(!isFinite(pcd[i]))
	continue;
      if(pcd[i].r < br.r - thresh || pcd[i].r > br.r + thresh ||
	 pcd[i].g < br.g - thresh || pcd[i].g > br.g + thresh ||
	 pcd[i].b < br.b - thresh || pcd[i].b > br.b + thresh)
	continue;

      brown->push_back(pcd[i]);
    }

    vw_.showCloud(brown);
    vw_.waitKey();
  }

  void TubeMeasurer::pointPickingCallback(const pcl::visualization::PointPickingEvent& event, void* cookie)
  {
    if(event.getPointIndex() == -1)
      return;
    
    Point pt;
    event.getPoint(pt.x, pt.y, pt.z);
    cout << "Selected point: " << pt.x << ", " << pt.y << ", " << pt.z << endl;
    selected_.push_back(pt);
    if(selected_.size() == 2) { 
      std::stringstream ss;
      ss << selected_[0] << selected_[1];
      vw_.vis_.addArrow<Point, Point>(selected_[0], selected_[1], 0.0, 0.0, 1.0, ss.str());

      visible_pairs_.push_back(selected_);
      selected_.clear();
    }
  }

  void TubeMeasurer::incrementIntrinsics(double dfx, double dfy, double dcx, double dcy)
  {
    proj_.fx_ += dfx;
    proj_.fy_ += dfy;
    proj_.cx_ += dcx;
    proj_.cy_ += dcy;
    cout << "Intrinsics: fx = " << proj_.fx_ << ", fy = " << proj_.fy_ << ", cx = " << proj_.cx_ << ", cy = " << proj_.cy_ << endl;
  }

  void TubeMeasurer::acceptVisible()
  {
    for(size_t i = 0; i < visible_pairs_.size(); ++i) {
      ROS_ASSERT(visible_pairs_[i].size() == 2);
      std::pair<ProjectedPoint, ProjectedPoint> ppts;
      proj_.project(visible_pairs_[i][0], &ppts.first);
      proj_.project(visible_pairs_[i][1], &ppts.second);
      accepted_.push_back(ppts);
    }
    cout << "Accepted " << visible_pairs_.size() << " pairs of points." << endl;
    
    clearSelection();
  }

  void TubeMeasurer::clearSelection()
  {
    selected_.clear();
    visible_pairs_.clear();
    vw_.vis_.removeAllShapes();
  }
  
  void TubeMeasurer::increment(int num)
  {
    clearSelection();
    
    idx_ += num;
    if(idx_ < 0)
      idx_ = 0;
    if(idx_ >= (int)sseq_.size())
      idx_ = sseq_.size() - 1;
  }

  void TubeMeasurer::saveAccepted() const
  {
    string p = path_;
    if(p[p.size() - 1] == '/')
      p = p.substr(0, p.size() - 1);
    p += "-accepted.txt";
    cout << "Saving accepted points to " << p << endl;
    accepted_.save(p);
  }

  void TubeMeasurer::loadAccepted()
  {
    string p = path_;
    if(p[p.size() - 1] == '/')
      p = p.substr(0, p.size() - 1);
    p += "-accepted.txt";
    cout << "Loading accepted points from " << p << endl;
    accepted_.load(p);

    cout << "New accepted points: " << endl;
    cout << accepted_;
  }

  void TubeMeasurer::gridSearch()
  {
    if(accepted_.empty()) {
      cout << "You must choose some pairs first." << endl;
      return;
    }
      
    GridSearch gs(4);
    gs.verbose_ = false;
    gs.objective_ = LossFunction::Ptr(new LossFunction(accepted_, actual_distance_));
    gs.num_scalings_ = 7;
    gs.max_resolutions_ << 50, 50, 50, 50;
    gs.grid_radii_ << 4, 4, 4, 4;
    gs.scale_factors_ << 0.5, 0.5, 0.5, 0.5;
    gs.couplings_ << 0, 0, 0, 0;

    ArrayXd init(4);
    init << proj_.fx_, proj_.fy_, proj_.cx_, proj_.cy_;
    cout << "Loss before optimization: " << gs.objective_->eval(init) << endl;
    ArrayXd xstar = gs.search(init);

    cout << "Got intrinsics of: " << xstar.transpose() << endl;
    proj_.fx_ = xstar(0);
    proj_.fy_ = xstar(1);
    proj_.cx_ = xstar(2);
    proj_.cy_ = xstar(3);
    cout << "Best loss: " << gs.objective_->eval(xstar) << endl;
  }
  
  LossFunction::LossFunction(const AcceptedPoints& accepted,
			     double actual_distance) :
    accepted_(accepted),
    actual_distance_(actual_distance)
  {
  }
  
  double LossFunction::eval(const Eigen::VectorXd& x) const
  {
    ROS_ASSERT(x.rows() == 4);
    
    double loss = 0;
    for(size_t i = 0; i < accepted_.size(); ++i) {
      Point pt0;
      Point pt1;
      Projector::project(x(0), x(1), x(2), x(3), accepted_[i].first, &pt0);
      Projector::project(x(0), x(1), x(2), x(3), accepted_[i].second, &pt1);
      double dist = pcl::euclideanDistance(pt0, pt1);
      loss += fabs(dist - actual_distance_);
    }

    return loss / (double)accepted_.size();
  }
  
} // namespace rgbd
