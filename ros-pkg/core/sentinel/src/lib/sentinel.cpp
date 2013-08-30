#include <sentinel/sentinel.h>
#include <ros/assert.h>
#include <ros/console.h>
#include <boost/filesystem.hpp>

using namespace std;
namespace bfs = boost::filesystem;

cv::Vec3b colorize(double depth, double min_range, double max_range)
{
  if(depth == 0)
    return cv::Vec3b(0, 0, 0);
    
  double increment = (max_range - min_range) / 3;
  double thresh0 = min_range;
  double thresh1 = thresh0 + increment;
  double thresh2 = thresh1 + increment;
  double thresh3 = thresh2 + increment;
    
  if(depth < thresh0) {
    return cv::Vec3b(0, 0, 255);
  }
  if(depth >= thresh0 && depth < thresh1) {
    int val = (depth - thresh0) / (thresh1 - thresh0) * 255.;
    return cv::Vec3b(val, val, 255 - val);
  }
  else if(depth >= thresh1 && depth < thresh2) {
    int val = (depth - thresh1) / (thresh2 - thresh1) * 255.;
    return cv::Vec3b(255, 255 - val, 0);
  }
  else if(depth >= thresh2 && depth < thresh3) {
    int val = (depth - thresh2) / (thresh3 - thresh2) * 255.;
    return cv::Vec3b(255 - val, val, 0);
  }
    
  return cv::Vec3b(0, 255, 0);
}
  
cv::Mat3b falseColor(const DepthMat& depth)
{
  cv::Mat3b cvimg(depth.rows(), depth.cols());
  cvimg = cv::Vec3b(0, 0, 0);
  for(int y = 0; y < cvimg.rows; ++y)
    for(int x = 0; x < cvimg.cols; ++x)
      cvimg(y, x) = colorize(depth.coeffRef(y, x) * 0.001, 0, 10);
  return cvimg;
}

Sentinel::Sentinel(double update_interval,
                   int max_training_imgs,
                   double threshold,
                   bool visualize,
                   OpenNI2Interface::Resolution color_res,
                   OpenNI2Interface::Resolution depth_res) :
  update_interval_(update_interval),
  max_training_imgs_(max_training_imgs),
  threshold_(threshold),
  visualize_(visualize),
  oni_(color_res, depth_res)
{
  oni_.setHandler(this);
  if(depth_res == OpenNI2Interface::VGA)
    model_ = boost::shared_ptr<BackgroundModel>(new BackgroundModel(640, 480, 16, 12, 0.3, 7, 0.2));
  else if(depth_res == OpenNI2Interface::QVGA)
    model_ = boost::shared_ptr<BackgroundModel>(new BackgroundModel(320, 240, 8, 6, 0.3, 7, 0.2));
  else {
    ROS_ASSERT(0);
  }
}

void Sentinel::run()
{
  update_timer_.start();
  oni_.run();
}

void Sentinel::rgbdCallback(openni::VideoFrameRef oni_color, openni::VideoFrameRef oni_depth)
{
  #if JARVIS_DEBUG
  ScopedTimer st("Sentinel::rgbdCallback");
  #endif

  double image_timestamp = (double)oni_color.getTimestamp() * 1e-6;
  double depth_timestamp = (double)oni_depth.getTimestamp() * 1e-6;
  timespec clk;
  clock_gettime(CLOCK_REALTIME, &clk);
  double callback_timestamp = clk.tv_sec + clk.tv_nsec * 1e-9;

  double thresh = 1.1 * (1.0 / 60.0);
  if(fabs(depth_timestamp - image_timestamp) > thresh) {
    ROS_WARN_STREAM("rgbdCallback got an rgbd pair with timestamp delta of "
                    << depth_timestamp - image_timestamp);
  }

  process(oni_color, oni_depth, callback_timestamp);
}

void Sentinel::process(openni::VideoFrameRef color,
                       openni::VideoFrameRef depth,
                       double ts)
{
  // -- Update model.
  if(update_timer_.getSeconds() > update_interval_) {
    update_timer_.reset();
    update_timer_.start();
    
    updateModel(depth);
  }

  #if JARVIS_DEBUG
  ScopedTimer st("Sentinel::process after update");
  #endif
  
  // -- If the model has been trained suffificiently, make predictions.
  if((int)training_.size() == 0)
    return;
  
  if((int)mask_.size() != depth.getHeight() * depth.getWidth())
    mask_.resize(depth.getHeight() * depth.getWidth());
  memset(&mask_[0], 0, mask_.size());

  // -- Get raw mask.
  size_t num_fg = 0;
  {
    #if JARVIS_DEBUG
    ScopedTimer st("Getting raw mask");
    #endif
    
    num_fg = model_->predict(depth, &mask_);
  }

  // -- Process the detection.
  if((double)num_fg / mask_.size() > threshold_) {
    handleDetection(color, depth, mask_, num_fg, ts);
  }
  
  // -- Visualize.
  if(visualize_) {
    if(vis_.rows != color.getHeight())
      vis_ = cv::Mat3b(cv::Size(color.getWidth(), color.getHeight()));

    // For now, you can only use this option if the color
    // and depth streams are of the same size.
    ROS_ASSERT(vis_.rows * vis_.cols == (int)mask_.size());
    
    oniToCV(color, vis_);
    for(int y = 0; y < vis_.rows; ++y)
      for(int x = 0; x < vis_.cols; ++x)
        if(mask_[y * vis_.cols + vis_.cols - x - 1] == 255)
          vis_(y, x)[2] = 255;
          //cv::circle(vis_, cv::Point(x, y), 2, cv::Scalar(0, 0, 255), -1);
    
    cv::imshow("Sentinel", vis_);
    cv::imshow("Depth", falseColor(oniDepthToEigen(depth)));
    char key = cv::waitKey(2);
    if(key == 'q')
      oni_.terminate();
  }
}

void Sentinel::updateModel(openni::VideoFrameRef depth)
{
  #if JARVIS_DEBUG
  ScopedTimer st("Sentinel::updateModel");
  #endif
  
  cout << "Updating model." << endl;
  training_.push(depth);
  model_->increment(depth);
  
  if((int)training_.size() > max_training_imgs_) { 
    model_->increment(training_.front(), -1);
    training_.pop();
  }
}

DiskStreamingSentinel::DiskStreamingSentinel(std::string dir,
                                             double save_interval,
                                             double update_interval,
                                             int max_training_imgs,
                                             double threshold,
                                             bool visualize,
                                             OpenNI2Interface::Resolution color_res,
                                             OpenNI2Interface::Resolution depth_res) :
  Sentinel(update_interval, max_training_imgs, threshold, visualize, color_res, depth_res),
  dir_(dir),
  save_interval_(save_interval)
{
  save_timer_.start();
}

void DiskStreamingSentinel::save(cv::Mat3b color, DepthMatConstPtr depth,
                                 cv::Mat3b vis, double ts) const
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
  cv::imwrite(depthpath.str(), falseColor(*depth));
  
  ostringstream imagepath;
  imagepath << imagedir << "/" << buffer << ":";
  imagepath << setiosflags(ios::fixed) << setprecision(3) << ts - floor(ts);
  imagepath << "-image.jpg";
  cout << "Saving to " << imagepath.str() << endl;
  cv::imwrite(imagepath.str(), color);

  string color_symlink_path = dir_ + "/recent_color_image.jpg";
  if(bfs::exists(color_symlink_path)) {
    cout << "exists.  removing." << endl;
    bfs::remove(color_symlink_path);
  }
  ROS_ASSERT(!bfs::exists(color_symlink_path));
  bfs::create_symlink(imagepath.str().substr(dir_.size() + 1), color_symlink_path);
}

cv::Mat1b DiskStreamingSentinel::depthMatToCV(const DepthMat& depth) const
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

void DiskStreamingSentinel::handleDetection(openni::VideoFrameRef color,
                                            openni::VideoFrameRef depth,
                                            const std::vector<uint8_t>& mask,
                                            size_t num_fg, double timestamp)
{
  if(save_timer_.getSeconds() < save_interval_)
    return;

  #if JARVIS_DEBUG
  ScopedTimer st("Saving");
  #endif
  
  if(!bfs::exists(dir_))
    bfs::create_directory(dir_);
    
  save(oniToCV(color), oniDepthToEigenPtr(depth), vis_, timestamp);
  save_timer_.reset();
  save_timer_.start();
}

ROSStreamingSentinel::ROSStreamingSentinel(double update_interval,
                                           int max_training_imgs,
                                           double threshold,
                                           bool visualize,
                                           OpenNI2Interface::Resolution color_res,
                                           OpenNI2Interface::Resolution depth_res) :
  Sentinel(update_interval, max_training_imgs, threshold, visualize, color_res, depth_res)
{
  pub_ = nh_.advertise<sentinel::Detection>("detections", 1000);

  if(depth_res == OpenNI2Interface::QVGA) {
    msg_.indices.reserve(320*240);
    msg_.depth.reserve(320*240);
  }
  else if(depth_res == OpenNI2Interface::VGA) {
    msg_.indices.reserve(640*480);
    msg_.depth.reserve(640*480);
  }
  else
    ROS_ASSERT(0);

  if(color_res == OpenNI2Interface::QVGA) {
    msg_.color.reserve(320*240*3);
  }
  else if(color_res == OpenNI2Interface::VGA) {
    msg_.color.reserve(640*480*3);
  }
  else
    ROS_ASSERT(0);
}

void ROSStreamingSentinel::handleDetection(openni::VideoFrameRef color,
                                           openni::VideoFrameRef depth,
                                           const std::vector<uint8_t>& mask,
                                           size_t num_fg, double timestamp)
{
  #if JARVIS_DEBUG
  ScopedTimer st("ROSStreamingSentinel::handleDetection - total");
  #endif

  if(!ros::ok()) {
    oni_.terminate();
    return;
  }

  ROS_ASSERT(color.getHeight() == depth.getHeight());
  
  msg_.indices.resize(num_fg);
  msg_.depth.resize(num_fg);
  msg_.color.resize(num_fg * 3);  // RGB

  uint8_t* color_data = (uint8_t*)color.getData();
  uint16_t* depth_data = (uint16_t*)depth.getData();
  
  size_t idx = 0;
  for(size_t i = 0; i < mask.size(); ++i) {
    if(mask[i]) {
      msg_.indices[idx] = i;
      msg_.depth[idx] = depth_data[i];
      msg_.color[idx*3+0] = color_data[i*3+0];
      msg_.color[idx*3+1] = color_data[i*3+1];
      msg_.color[idx*3+2] = color_data[i*3+2];
      ++idx;
    }
  }

  pub_.publish(msg_);
}



