#include <rgbd_sequence/intrinsics_visualizer.h>
#include <pcl/common/distances.h>

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

  IntrinsicsVisualizer::IntrinsicsVisualizer() :
    idx_(0),
    pcd_(new Cloud),
    show_color_(true)
  {
    vw_.vis_.registerPointPickingCallback(&IntrinsicsVisualizer::pointPickingCallback, *this);
  }
  
  void IntrinsicsVisualizer::decolorize(rgbd::Cloud* pcd) const
  {
    for(size_t i = 0; i < pcd->size(); ++i) {
      pcd->at(i).r = 127;
      pcd->at(i).g = 127;
      pcd->at(i).b = 127;
    }
  }

  void IntrinsicsVisualizer::run(const std::string& path, double actual_distance)
  {
    path_ = path;
    actual_distance_ = actual_distance;
    
    sseq_.load(path);
    model_ = sseq_.model_;
    Frame frame;
    while(true) {
      sseq_.readFrame(idx_, &frame);
      model_.frameToCloud(frame, pcd_.get());
      ROS_DEBUG_STREAM("IntrinsicsVisualizer using model: " << endl << model_.status("  "));
      if(!show_color_)
        decolorize(pcd_.get());

      vw_.showCloud(pcd_);
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
      case 'C':
        show_color_ = !show_color_;
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
      default:
        break;
      }
    }
  }
  
  void IntrinsicsVisualizer::pointPickingCallback(const pcl::visualization::PointPickingEvent& event, void* cookie)
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
      vw_.vis_.addSphere<Point>(selected_[0], 0.05, ss.str() + "-point0");
      vw_.vis_.addSphere<Point>(selected_[1], 0.05, ss.str() + "-point1");
      
      visible_pairs_.push_back(selected_);
      selected_.clear();
    }
  }

  void IntrinsicsVisualizer::incrementIntrinsics(double dfx, double dfy, double dcx, double dcy)
  {
    model_.fx_ += dfx;
    model_.fy_ += dfy;
    model_.cx_ += dcx;
    model_.cy_ += dcy;
    cout << "PrimeSenseModel: " << endl;
    cout << model_.status("  ");
  }

  void IntrinsicsVisualizer::acceptVisible()
  {
    for(size_t i = 0; i < visible_pairs_.size(); ++i) {
      ROS_ASSERT(visible_pairs_[i].size() == 2);
      std::pair<ProjectivePoint, ProjectivePoint> ppts;
      model_.project(visible_pairs_[i][0], &ppts.first);
      model_.project(visible_pairs_[i][1], &ppts.second);
      accepted_.push_back(ppts);
    }
    cout << "Accepted " << visible_pairs_.size() << " pairs of points." << endl;
    
    clearSelection();
  }

  void IntrinsicsVisualizer::clearSelection()
  {
    selected_.clear();
    visible_pairs_.clear();
    vw_.vis_.removeAllShapes();
  }
  
  void IntrinsicsVisualizer::increment(int num)
  {
    clearSelection();
    
    idx_ += num;
    if(idx_ < 0)
      idx_ = 0;
    if(idx_ >= (int)sseq_.size())
      idx_ = sseq_.size() - 1;
  }

  void IntrinsicsVisualizer::saveAccepted() const
  {
    string p = path_;
    if(p[p.size() - 1] == '/')
      p = p.substr(0, p.size() - 1);
    p += "-accepted.txt";
    cout << "Saving accepted points to " << p << endl;
    accepted_.save(p);
  }

  void IntrinsicsVisualizer::loadAccepted()
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

  void IntrinsicsVisualizer::gridSearch()
  {
    if(accepted_.empty()) {
      cout << "You must choose some pairs first." << endl;
      return;
    }
      
    GridSearch gs(4);
    gs.verbose_ = false;
    gs.objective_ = LossFunction::Ptr(new LossFunction(accepted_, actual_distance_, model_.width_, model_.height_));
    gs.num_scalings_ = 7;
    gs.max_resolutions_ << 50, 50, 50, 50;
    gs.grid_radii_ << 4, 4, 4, 4;
    gs.scale_factors_ << 0.5, 0.5, 0.5, 0.5;
    gs.couplings_ << 0, 0, 0, 0;

    ArrayXd init(4);
    init << model_.fx_, model_.fy_, model_.cx_, model_.cy_;
    cout << "Loss before optimization: " << gs.objective_->eval(init) << endl;
    printErrors(init);
    ArrayXd xstar = gs.search(init);

    cout << "Got intrinsics of: " << xstar.transpose() << endl;
    model_.fx_ = xstar(0);
    model_.fy_ = xstar(1);
    model_.cx_ = xstar(2);
    model_.cy_ = xstar(3);
    cout << "Best loss: " << gs.objective_->eval(xstar) << endl;
    printErrors(xstar);
  }

  void IntrinsicsVisualizer::printErrors(const Eigen::ArrayXd& x) const
  {
    ROS_ASSERT(x.rows() == 4);

    PrimeSenseModel model;
    model.width_ = model_.width_;
    model.height_ = model_.height_;
    model.fx_ = x(0);
    model.fy_ = x(1);
    model.cx_ = x(2);
    model.cy_ = x(3);

    for(size_t i = 0; i < accepted_.size(); ++i) {
      Point pt0;
      Point pt1;
      model.project(accepted_[i].first, &pt0);
      model.project(accepted_[i].second, &pt1);
      double dist = pcl::euclideanDistance(pt0, pt1);
      cout << "Error " << i << ": " << fabs(dist - actual_distance_) << endl;
    }
  }
  
  LossFunction::LossFunction(const AcceptedPoints& accepted,
                             double actual_distance, int width, int height) :
    accepted_(accepted),
    actual_distance_(actual_distance),
    width_(width),
    height_(height)
  {
  }
  
  double LossFunction::eval(const Eigen::VectorXd& x) const
  {
    ROS_ASSERT(x.rows() == 4);

    PrimeSenseModel model;
    model.width_ = width_;
    model.height_ = height_;
    model.fx_ = x(0);
    model.fy_ = x(1);
    model.cx_ = x(2);
    model.cy_ = x(3);
    
    double loss = 0;
    for(size_t i = 0; i < accepted_.size(); ++i) {
      Point pt0;
      Point pt1;
      model.project(accepted_[i].first, &pt0);
      model.project(accepted_[i].second, &pt1);
      double dist = pcl::euclideanDistance(pt0, pt1);
      loss += fabs(dist - actual_distance_);
    }

    return loss / (double)accepted_.size();
  }
  
} // namespace rgbd
