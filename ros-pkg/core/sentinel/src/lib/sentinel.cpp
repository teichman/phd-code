#include <sentinel/sentinel.h>

using namespace std;
namespace bfs = boost::filesystem;
using namespace rgbd;

Sentinel::Sentinel(std::string name,
                   double update_interval,
                   double save_interval,
                   int max_training_imgs,
                   double threshold,
                   const std::string& device_id,
                   pcl::OpenNIGrabber::Mode mode) :
  grabber_(device_id, mode, mode),
  model_((mode == pcl::OpenNIGrabber::OpenNI_QQVGA_30Hz) ? 320*240 : 640*480),
  update_interval_(update_interval),
  save_interval_(save_interval),
  max_training_imgs_(max_training_imgs),
  threshold_(threshold),
  dir_(".sentinel-" + name)
{
  boost::function<void(const boost::shared_ptr<openni_wrapper::Image>&,
                       const boost::shared_ptr<openni_wrapper::DepthImage>&,
                       float)> rgbd_cb;
  rgbd_cb = boost::bind(&Sentinel::rgbdCallback, this, _1, _2, _3);
  grabber_.registerCallback(rgbd_cb);

  //grabber_.registerCallback(boost::bind(&Sentinel::rgbdCallback, this, _1, _2, _3));
}

void Sentinel::run()
{
  if(!bfs::exists(dir_))
    bfs::create_directory(dir_);
  
  grabber_.start();
  update_timer_.start();
  save_timer_.start();
  while(true)
    usleep(1e6);
  grabber_.stop();
}

void Sentinel::rgbdCallback(const boost::shared_ptr<openni_wrapper::Image>& oni_rgb,
                            const boost::shared_ptr<openni_wrapper::DepthImage>& oni_depth,
                            float f_inv)
{
  double depth_timestamp = (double)oni_depth->getTimeStamp() / (double)1e6;
  double image_timestamp = (double)oni_rgb->getTimeStamp() / (double)1e6;
  timespec clk;
  clock_gettime(CLOCK_REALTIME, &clk);
  double callback_timestamp = clk.tv_sec + clk.tv_nsec * 1e-9;

  double thresh = 1.1 * (1.0 / 60.0);
  if(fabs(depth_timestamp - image_timestamp) > thresh) {
    ROS_WARN_STREAM("rgbdCallback got an rgbd pair with timestamp delta of "
                    << depth_timestamp - image_timestamp);
  }

  DepthMatPtr depth = StreamRecorder::oniDepthToEigenPtr(oni_depth);
  cv::Mat3b img = StreamRecorder::oniToCV(oni_rgb);
  process(depth, img, callback_timestamp);
}

void Sentinel::process(DepthMatConstPtr depth, cv::Mat3b img, double ts)
{
  // cout << "Processing." << endl;
  // cout << img.rows << " " << img.cols << " " << depth->rows() << " " << depth->cols() << endl;
  ROS_ASSERT(depth->rows() == img.rows);
  ROS_ASSERT(depth->cols() == img.cols);
  
  // -- Update model.
  if(update_timer_.getSeconds() > update_interval_) {
    update_timer_.reset();
    update_timer_.start();
    
    updateModel(depth);
  }
  
  // -- If the model has been trained suffificiently, make predictions.
  if((int)training_.size() == 0)
    return;
  
  if(mask_.rows != img.rows)
    mask_ = cv::Mat1b(img.size());

  // -- Get raw mask.
  mask_ = 0;
  int idx = 0;
  for(int y = 0; y < depth->rows(); ++y) {
    for(int x = 0; x < depth->cols(); ++x, ++idx) {
      if(depth->coeff(y, x) == 0)
        continue;
      if(!model_.isBackground(idx, depth->coeff(y, x) / 1000.0))
        mask_(y, x) = 255;
    }
  }

  // -- Get rid of noise.
  cv::erode(mask_, mask_, cv::Mat(), cv::Point(-1, -1), 4);
  cv::dilate(mask_, mask_, cv::Mat(), cv::Point(-1, -1), 4);

  // -- Count up.
  vis_ = img.clone();
  double num_fg = 0;
  for(int y = 0; y < depth->rows(); ++y) { 
    for(int x = 0; x < depth->cols(); ++x, ++idx) { 
      if(mask_(y, x) == 255) { 
        ++num_fg;
        vis_(y, x)[2] = 255;
      }
    }
  }

  // -- Save.
  double total = depth->rows() * depth->cols();
  if(num_fg / total > threshold_) {
    if(save_timer_.getSeconds() > save_interval_) { 
      cout << "Saving.  pct fg: " << num_fg / total << endl;
      save(depth, img, vis_, ts);
      save_timer_.reset();
      save_timer_.start();
    }
  }
  
  // -- Visualize.
  cv::imshow("Sentinel", vis_);
  cv::waitKey(2);
}

void Sentinel::updateModel(DepthMatConstPtr depth)
{
  cout << "Updating model." << endl;
  ROS_ASSERT(depth);
  
  training_.push(depth);
  model_.increment(*depth);
  
  if((int)training_.size() > max_training_imgs_) { 
    model_.increment(*training_.front(), -1);
    training_.pop();
  }

  model_.finalize();
}

void Sentinel::save(DepthMatConstPtr depth, cv::Mat3b img, cv::Mat3b vis, double ts) const
{
  time_t rawtime = ts;
  struct tm* timeinfo;
  char buffer[80];
  timeinfo = localtime(&rawtime);
  strftime(buffer, 80, "%Y-%m-%d", timeinfo);

  string datedir = dir_ + "/" + buffer;
  string depthdir = datedir + "/depth";
  string imagedir = datedir + "/image";
  if(!bfs::exists(datedir))
    bfs::create_directory(datedir);
  if(!bfs::exists(depthdir))
    bfs::create_directory(depthdir);
  if(!bfs::exists(imagedir))
    bfs::create_directory(imagedir);

  strftime(buffer, 80, "%H:%M:%S", timeinfo);
  ostringstream depthpath;
  depthpath << depthdir << "/" << buffer << ":";
  depthpath << setiosflags(ios::fixed) << setprecision(3) << ts - floor(ts);
  depthpath << "-depth.png";
  cout << "Saving to " << depthpath.str() << endl;
  cv::imwrite(depthpath.str(), depthMatToCV(*depth));
  
  ostringstream imagepath;
  imagepath << imagedir << "/" << buffer << ":";
  imagepath << setiosflags(ios::fixed) << setprecision(3) << ts - floor(ts);
  imagepath << "-image.jpg";
  cout << "Saving to " << imagepath.str() << endl;
  cv::imwrite(imagepath.str(), img);
  
}

cv::Mat1b Sentinel::depthMatToCV(const DepthMat& depth) const
{
  cv::Mat1b vis(cv::Size(depth.cols(), depth.rows()), 0);
  double max_dist = 7.5;

  for(int y = 0; y < vis.rows; ++y) {
    for(int x = 0; x < vis.cols; ++x) {
      if(depth(y, x) != 0)
        vis(y, x) = 255 * (1.0 - fmin(max_dist, depth(y, x) / 1000.0) / max_dist);
    }
  }

  return vis;
}
