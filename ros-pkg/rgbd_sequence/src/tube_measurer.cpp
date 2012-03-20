#include <rgbd_sequence/tube_measurer.h>

namespace rgbd
{

  TubeMeasurer::TubeMeasurer() :
    idx_(0),
    warped_(new Cloud)
  {
    vw_.vis_.registerPointPickingCallback(&TubeMeasurer::pointPickingCallback, *this);
  }
  
  void TubeMeasurer::run(const std::string& path)
  {
    sseq_.load(path);
    proj_.fx_ = sseq_.fx_;
    proj_.fy_ = sseq_.fy_;
    proj_.cx_ = sseq_.cx_;
    proj_.cy_ = sseq_.cy_;
    
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
  
  void TubeMeasurer::increment(int num)
  {
    vw_.vis_.removeAllShapes();
    
    idx_ += num;
    if(idx_ < 0)
      idx_ = 0;
    if(idx_ >= (int)sseq_.size())
      idx_ = sseq_.size() - 1;
  }
  
} // namespace rgbd
