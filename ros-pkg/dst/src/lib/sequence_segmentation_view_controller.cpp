#include <dst/sequence_segmentation_view_controller.h>

using namespace std;

#define NUM_THREADS (getenv("NUM_THREADS") ? atoi(getenv("NUM_THREADS")) : 1)

namespace dst
{

  SequenceSegmentationViewController::SequenceSegmentationViewController(KinectSequence::Ptr seq) :
    OpenCVViewDelegate(),
    Lockable(),
    seq_(seq),
    seg_view_("Segmentation"),
    pcd_view_("Cloud"),
    img_view_("Image"),
    seed_radius_(0),
    sp_(NUM_THREADS),
    current_idx_(0),
    quitting_(false),
    needs_redraw_(true),
    state_(RAW),
    show_depth_(true),
    show_seg_3d_(false),
    max_range_(3.0)
  {
    img_view_.setDelegate((OpenCVViewDelegate*)this);
    img_view_.message_scale_ = 0.25;
    img_view_.message_thickness_ = 1.0;
    
    segmented_pcds_.resize(seq->images_.size());
    for(size_t i = 0; i < segmented_pcds_.size(); ++i)
      segmented_pcds_[i] = KinectCloud::Ptr(new KinectCloud());
  }

  SequenceSegmentationViewController::~SequenceSegmentationViewController()
  {
  }
  
  void SequenceSegmentationViewController::run()
  {
    ROS_ASSERT(seq_);
    seed_vis_ = seq_->images_[current_idx_].clone();
    seg_vis_ = cv::Mat3b(seq_->images_[current_idx_].size(), 127);
    
    while(true) {
      if(needs_redraw_)
	draw();

      char key = img_view_.cvWaitKey(30);
      handleKeypress(key);

      if(quitting_)
	break;
    }
  }

  void SequenceSegmentationViewController::increaseSeedWeights()
  {
    ROS_INFO_STREAM("Increasing seed weights.  Assuming seed weight is the 0th node weight.");
    Eigen::VectorXd w = sp_.getWeights();
    w(sp_.getEdgeWeights().rows()) *= 10.0;
    sp_.setWeights(w);
  }
  
  void SequenceSegmentationViewController::useSegmentationAsSeed()
  {
    seq_->seed_images_[current_idx_] = seq_->segmentations_[current_idx_].clone();
    // cv::Mat1b seed = seq_->seed_images_[current_idx_];
    // for(int y = 0; y < seed.rows; ++y) {
    //   for(int x = 0; x < seed.cols; ++x) {
    // 	if(seed(y, x) == 127)
    // 	  seed(y, x) = 0;
    needs_redraw_ = true;
  }
  
  void SequenceSegmentationViewController::toggleDebug()
  {
    sp_.toggleDebug();
    
    if(sp_.getDebug())
      cout << "Debug mode is on." << endl;
    else
      cout << "Debug mode is off." << endl;
  }
  
  void SequenceSegmentationViewController::saveGraphviz() const
  {
    string filename = "segmentation_pipeline_graphviz";
    ofstream f;
    f.open(filename.c_str());
    f << sp_.getGraphviz();
    f.close();

    cout << "Saved pipeline graphviz to " << filename << endl;
  }
  
  void SequenceSegmentationViewController::handleKeypress(char key)
  {
    lock();
    
    // -- Global keys.
    string retval;
    switch(key) {
    case 'q':
      cout << "Really quit? " << endl;
      cin >> retval;
      if(retval.compare("y") == 0)
	quitting_ = true;
      break;
    case 'j':
      advance(-1);
      break;
    case 'k':
      advance(1);
      break;
    case 'F':
      increaseSeedWeights();
      break;
    case 'l':
      transitionTo(SEED);
      break;
    case 'r':
      transitionTo(RAW);
      break;
    case 'v':
      saveGraphviz();
      break;
    case 'p':
      show_seg_3d_ = !show_seg_3d_;
      needs_redraw_ = true;
      break;
    case '[':
      max_range_ -= 0.33;
      needs_redraw_ = true;
      break;
    case ']':
      max_range_ += 0.33;
      needs_redraw_ = true;
      break;
    default:
      break;
    }

    // -- State-specific keys.
    switch(state_) {
    case SEED:
      switch(key) {
      case 'D':
	toggleDebug();
	break;
      case 'd':
	show_depth_ = !show_depth_;
	needs_redraw_ = true;
	break;
      case 'i':
	segmentImage();
	break;
      case 's':
	segmentSequence();
	break;
      case 'S':
	saveSequence();
	break;
      case 'L':
	cout << "Using segmentation as seed image." << endl;
	useSegmentationAsSeed();
	break;
      case '+':
	++seed_radius_;
	cout << "Seed radius: " << seed_radius_ << endl;
	break;
      case '-':
	--seed_radius_;
	if(seed_radius_ < 0)
	  seed_radius_ = 0;
	cout << "Seed radius: " << seed_radius_ << endl;
	break;
      case 'C':
	clearHelperSeedLabels();
	cout << "Cleared seed labels for all frames but the first." << endl;
	break;
      case 'c':
	seq_->seed_images_[current_idx_] = 127;
	needs_redraw_ = true;
      default:
	break;
      }
      break;
      
    case RAW:
      switch(key) {
      case 'v':
	cout << "Cloud is (wxh) " <<  seq_->pointclouds_[current_idx_]->width << " x "
	     << seq_->pointclouds_[current_idx_]->height << endl;
	cout << "Number of points: " << seq_->pointclouds_[current_idx_]->points.size() << endl;
	cout << "is_dense: " << seq_->pointclouds_[current_idx_]->is_dense << endl;
	cout << "sensor origin: " << seq_->pointclouds_[current_idx_]->sensor_origin_.transpose() << endl;
	break;
      default:
	break;
      }
    default:
      break;
    }
    
    unlock();
  }

  void SequenceSegmentationViewController::clearHelperSeedLabels()
  {
    for(size_t i = 1; i < seq_->seed_images_.size(); ++i)
      seq_->seed_images_[i] = 127;

    needs_redraw_ = true;
  }
  
  void SequenceSegmentationViewController::draw()
  {
    drawSegVis();
    seg_view_.updateImage(seg_vis_);
    if(show_seg_3d_ && !segmented_pcds_[current_idx_]->empty())
      pcd_view_.showCloud(segmented_pcds_[current_idx_]);
    else
      pcd_view_.showCloud(seq_->pointclouds_[current_idx_]);

    ostringstream oss;
    oss << "raw - " << current_idx_;
    switch(state_) {
    case RAW:
      
      img_view_.message_ = oss.str();
      img_view_.updateImage(seq_->images_[current_idx_]);
      break;
    case SEED:
      img_view_.message_ = "";
      drawSeedVis();
      img_view_.updateImage(seed_vis_);
    default:
      break;
    }

    needs_redraw_ = false;
  }

  void SequenceSegmentationViewController::drawSegVis()
  {
    for(int y = 0; y < seg_vis_.rows; ++y) {
      for(int x = 0; x < seg_vis_.cols; ++x) {
	switch(seq_->segmentations_[current_idx_](y, x)) {
	case 127:
	  seg_vis_(y, x) = cv::Vec3b(127, 127, 127);
	  break;
	case 0:
	  seg_vis_(y, x) = cv::Vec3b(0, 0, 0);
	  break;
	case 255:
	  seg_vis_(y, x) = cv::Vec3b(255, 255, 255); //seq_->images_[current_idx_](y, x);
	  break;
	default:
	  break;
	}
      }
    }
  }
  
  void SequenceSegmentationViewController::drawSeedVis()
  {
    cv::Mat3b vis = seq_->images_[current_idx_];
    if(show_depth_)
      vis = sp_.getZBuffer(*seq_->pointclouds_[current_idx_], 0, 0.5, max_range_);
    
    for(int y = 0; y < seed_vis_.rows; ++y) {
      for(int x = 0; x < seed_vis_.cols; ++x) {
	switch(seq_->seed_images_[current_idx_](y, x)) {
	case 127:
	  seed_vis_(y, x) = vis(y, x);
	  break;
	case 0:
	  seed_vis_(y, x) = cv::Vec3b(0, 0, 0);
	  break;
	case 255:
	  seed_vis_(y, x) = cv::Vec3b(255, 255, 255);
	  break;
	default:
	  break;
	}
      }
    }
  }

  void SequenceSegmentationViewController::segmentImage()
  {
    sp_.reset();
    sp_.run(seq_->seed_images_[current_idx_],
	    seq_->images_[current_idx_],
	    seq_->pointclouds_[current_idx_],
	    cv::Mat3b(),
	    cv::Mat1b(),
	    KinectCloud::Ptr(),
	    seq_->segmentations_[current_idx_],
	    segmented_pcds_[current_idx_]);
    needs_redraw_ = true;
  }

  void SequenceSegmentationViewController::segmentSequence()
  {
    // -- Erase all segmentations except the previous one.
    for(size_t i = 0; i < seq_->segmentations_.size(); ++i) {
      if((int)i == current_idx_ - 1)
    	continue;
      seq_->segmentations_[i] = 127;
    }

    current_idx_ = 0;
    draw();
    cv::waitKey(10);
    
    sp_.reset();
    for(; current_idx_ < (int)seq_->images_.size(); ++current_idx_) {
      // First in the sequence doesn't get passed in 'prev' data.
      if(current_idx_ == 0) {
	sp_.run(seq_->seed_images_[current_idx_],
		seq_->images_[current_idx_],
		seq_->pointclouds_[current_idx_],
		cv::Mat3b(),
		cv::Mat1b(),
		KinectCloud::Ptr(),
		seq_->segmentations_[current_idx_],
		segmented_pcds_[current_idx_]);
      }
      else {
	sp_.run(seq_->seed_images_[current_idx_],
		seq_->images_[current_idx_],
		seq_->pointclouds_[current_idx_],
		seq_->images_[current_idx_-1],
		seq_->segmentations_[current_idx_-1],
		seq_->pointclouds_[current_idx_-1],
		seq_->segmentations_[current_idx_],
		segmented_pcds_[current_idx_]);
      }

      draw();
      int wait_time = 10;
      if(sp_.getDebug()) {
	cout << "Press s to stop, any other key to continue." << endl;
	//wait_time = 0;
	
	string filename;
	filename = generateFilename("debug", "segmented_pointcloud.pcd", 4);
	// Writer fails if there are no points?
	if(segmented_pcds_[current_idx_]->size() == 0) {
	  pcl::PointXYZRGB pt;
	  pt.x = 0; pt.y = 0; pt.z = -20;
	  segmented_pcds_[current_idx_]->push_back(pt);
	  segmented_pcds_[current_idx_]->push_back(pt);
	  segmented_pcds_[current_idx_]->push_back(pt);
	}
	pcl::io::savePCDFileBinary(filename, *segmented_pcds_[current_idx_]);
	filename = generateFilename("debug", "original_pointcloud.pcd", 4);
	pcl::io::savePCDFileBinary(filename, *seq_->pointclouds_[current_idx_]);
	filename = generateFilename("debug", "segmentation_mask.png", 4);
	cv::imwrite(filename, seq_->segmentations_[current_idx_]);
	filename = generateFilename("debug", "original_image.png", 4);
	cv::imwrite(filename, seq_->images_[current_idx_]);
	filename = generateFilename("debug", "segmented_image.png", 4);
	cv::imwrite(filename, seg_vis_);
      }
      
      char key = img_view_.cvWaitKey(wait_time);
      if(key == 's')
	break;
    }
    cout << "Sequence segmentation ended." << endl;

    if((size_t)current_idx_ == seq_->images_.size())
      current_idx_ = seq_->images_.size() - 1;
  }

  void SequenceSegmentationViewController::saveSequence()
  {
    bool flag = true;
    for(size_t i = 1; flag && i < seq_->seed_images_.size(); ++i)
      for(int y = 0; flag && y < seq_->seed_images_[i].rows; ++y)
	for(int x = 0; flag && x < seq_->seed_images_[i].cols; ++x)
	  if(seq_->seed_images_[i](y, x) != 127)
	    flag = false;
    
    if(!flag) { 
      cout << "This will clear all seed labels except those in the first frame." << endl;
      cout << "Continue?  (y/n): " << endl;
      string retval;
      cin >> retval;
      if(retval.compare("y") != 0) {
	cout << "Aborted." << endl;
	return;
      }
      clearHelperSeedLabels();
      draw();
      cv::waitKey(10);
    }
    
    cout << "Dir name for new sequence: " << endl;
    string dirname;
    cin >> dirname;
    seq_->save(dirname);
    cout << "Saved to " << dirname << endl;
  }
  
  void SequenceSegmentationViewController::advance(int num)
  {
    current_idx_ += num;
    if(current_idx_ < 0)
      current_idx_ = 0;
    if((size_t)current_idx_ >= seq_->images_.size())
      current_idx_ = seq_->images_.size() - 1;

    needs_redraw_ = true;
  }
  
  void SequenceSegmentationViewController::mouseEvent(int event, int x, int y, int flags, void* param)
  {
    //lock();
    if(state_ != SEED) {
      //unlock();
      return;
    }

    // -- Left click to add to source.
    if(flags & CV_EVENT_FLAG_LBUTTON) {
      for(int i = x - seed_radius_; i <= x + seed_radius_; ++i) { 
	for(int j = y - seed_radius_; j <= y + seed_radius_; ++j) {
	  if(i >= 0 && i < seed_vis_.cols &&
	     j >= 0 && j < seed_vis_.rows) {

	    seq_->seed_images_[current_idx_](j, i) = 255;
	  }
	}
      }
      needs_redraw_ = true;
    }

    // -- Right click to add to sink.
    else if(flags & CV_EVENT_FLAG_RBUTTON) {
      for(int i = x - seed_radius_; i <= x + seed_radius_; ++i) { 
	for(int j = y - seed_radius_; j <= y + seed_radius_; ++j) {
	  if(i >= 0 && i < seed_vis_.cols &&
	     j >= 0 && j < seed_vis_.rows) { 

	    seq_->seed_images_[current_idx_](j, i) = 0;
	  }
	}
      }
      needs_redraw_ = true;
    }

    //unlock();
  }
  
  void SequenceSegmentationViewController::transitionTo(state_t state) { 
    state_ = state;
    needs_redraw_ = true;
  }
  
} // namespace dst
