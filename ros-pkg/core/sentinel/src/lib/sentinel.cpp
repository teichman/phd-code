#include <sentinel/sentinel.h>
#include <ros/assert.h>
#include <ros/console.h>
#include <boost/filesystem.hpp>

using namespace std;
namespace bfs = boost::filesystem;
  
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
    model_ = boost::shared_ptr<BackgroundModel>(new BackgroundModel(640, 480, 16, 12, 0.1, MAX_DEPTH, 0.2));
  else if(depth_res == OpenNI2Interface::QVGA)
    model_ = boost::shared_ptr<BackgroundModel>(new BackgroundModel(320, 240, 8, 6, 0.1, MAX_DEPTH, 0.2));
  else {
    ROS_ASSERT(0);
  }
}

void Sentinel::run()
{
  update_timer_.start();
  oni_.run();
}

void Sentinel::rgbdCallback(openni::VideoFrameRef oni_color, openni::VideoFrameRef oni_depth,
                            size_t frame_id, double timestamp)
{
  #if JARVIS_DEBUG
  ScopedTimer st("Sentinel::rgbdCallback");
  #endif

  // -- Check for misaligned data.
  //    TODO: This should probably be in OpenNI2Interface.
  double image_timestamp = (double)oni_color.getTimestamp() * 1e-6;
  double depth_timestamp = (double)oni_depth.getTimestamp() * 1e-6;
  double thresh = 1.1 * (1.0 / 60.0);
  if(fabs(depth_timestamp - image_timestamp) > thresh) {
    ROS_WARN_STREAM("rgbdCallback got an rgbd pair with timestamp delta of "
                    << depth_timestamp - image_timestamp);
  }

  // -- Drop everything beyond MAX_DEPTH.
  uint16_t* data = (uint16_t*)oni_depth.getData();
  for(int y = 0; y < oni_depth.getHeight(); ++y)
    for(int x = 0; x < oni_depth.getWidth(); ++x)
      if(data[y * oni_depth.getWidth() + x] > MAX_DEPTH * 1000)  // very sporadic segfault here.  Why?
        data[y * oni_depth.getWidth() + x] = 0;

  process(oni_color, oni_depth, depth_timestamp, timestamp, frame_id);
}

void Sentinel::process(openni::VideoFrameRef color,
                       openni::VideoFrameRef depth,
                       double sensor_timestamp,
                       double wall_timestamp,
                       size_t frame_id)
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
  size_t num_in_mask = 0;
  {
    #if JARVIS_DEBUG
    ScopedTimer st("Getting raw mask");
    #endif
    
    num_in_mask = model_->predict(depth, &mask_);
  }

  // -- Process the detection.
  if((double)num_in_mask / mask_.size() > threshold_) {
    handleDetection(color, depth, mask_, num_in_mask, sensor_timestamp, wall_timestamp, frame_id);
  }
  handleNonDetection(color, depth, sensor_timestamp, wall_timestamp, frame_id);
  
  // -- Visualize.
  if(visualize_) {
    if(vis_.rows != color.getHeight())
      vis_ = cv::Mat3b(cv::Size(color.getWidth(), color.getHeight()));

    // For now, you can only use this option if the color
    // and depth streams are of the same size.
    ROS_ASSERT(vis_.rows * vis_.cols == (int)mask_.size());
    
    oniToCV(color, vis_);
    for(int y = 0; y < vis_.rows; ++y) {
      for(int x = 0; x < vis_.cols; ++x) {
        if(mask_[y * vis_.cols + vis_.cols - x - 1] == 255)
          vis_(y, x)[2] = 255;
        else if(mask_[y * vis_.cols + vis_.cols - x - 1] == 127)
          vis_(y, x)[1] = 255;
      }
    }
    
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
                                            size_t num_in_mask,
                                            double sensor_timestamp,
                                            double wall_timestamp,
                                            size_t frame_id)
{
  if(save_timer_.getSeconds() < save_interval_)
    return;

  #if JARVIS_DEBUG
  ScopedTimer st("Saving");
  #endif
  
  if(!bfs::exists(dir_))
    bfs::create_directory(dir_);
    
  save(oniToCV(color), oniDepthToEigenPtr(depth), vis_, wall_timestamp);
  save_timer_.reset();
  save_timer_.start();
}

ROSStreamingSentinel::ROSStreamingSentinel(string sensor_id,
                                           double update_interval,
                                           int max_training_imgs,
                                           double threshold,
                                           bool visualize,
                                           OpenNI2Interface::Resolution color_res,
                                           OpenNI2Interface::Resolution depth_res) :
  Sentinel(update_interval, max_training_imgs, threshold, visualize, color_res, depth_res),
  sensor_id_(sensor_id),
  bg_index_x_(0),
  bg_index_y_(0)
{
  fg_pub_ = nh_.advertise<sentinel::Foreground>("foreground", 1000);
  bg_pub_ = nh_.advertise<sentinel::Background>("background", 1000);
  initializeForegroundMessage();
  initializeBackgroundMessage();
}

void ROSStreamingSentinel::initializeForegroundMessage()
{
  fgmsg_.sensor_id = sensor_id_;
  fgmsg_.width = model_->width();
  fgmsg_.height = model_->height();
  fgmsg_.width_step = model_->widthStep();
  fgmsg_.height_step = model_->heightStep();

  fgmsg_.fg_indices.reserve(model_->size());
  fgmsg_.bg_fringe_indices.reserve(model_->size());
  
  if(oni_.depthRes() == OpenNI2Interface::QVGA) {
    fgmsg_.indices.reserve(320*240);
    fgmsg_.depth.reserve(320*240);
  }
  else if(oni_.depthRes() == OpenNI2Interface::VGA) {
    fgmsg_.indices.reserve(640*480);
    fgmsg_.depth.reserve(640*480);
  }
  else {
    ROS_ASSERT(0);
  }

  if(oni_.colorRes() == OpenNI2Interface::QVGA) {
    fgmsg_.color.reserve(320*240*3);
  }
  else if(oni_.colorRes() == OpenNI2Interface::VGA) {
    fgmsg_.color.reserve(640*480*3);
  }
  else {
    ROS_ASSERT(0);
  }
}

void ROSStreamingSentinel::initializeBackgroundMessage()
{
  bgmsg_.sensor_id = sensor_id_;
  bgmsg_.width = model_->width();
  bgmsg_.height = model_->height();
  
  if(oni_.depthRes() == OpenNI2Interface::QVGA) {
    bgmsg_.indices.reserve(320*240);
    bgmsg_.depth.reserve(320*240);
  }
  else if(oni_.depthRes() == OpenNI2Interface::VGA) {
    bgmsg_.indices.reserve(640*480);
    bgmsg_.depth.reserve(640*480);
  }
  else {
    ROS_ASSERT(0);
  }

  if(oni_.colorRes() == OpenNI2Interface::QVGA) {
    bgmsg_.color.reserve(320*240*3);
  }
  else if(oni_.colorRes() == OpenNI2Interface::VGA) {
    bgmsg_.color.reserve(640*480*3);
  }
  else {
    ROS_ASSERT(0);
  }
}

void ROSStreamingSentinel::handleNonDetection(openni::VideoFrameRef color,
                                              openni::VideoFrameRef depth,
                                              double sensor_timestamp,
                                              double wall_timestamp,
                                              size_t frame_id)
{
  #if JARVIS_DEBUG
  ScopedTimer st("ROSStreamingSentinel::handleNonDetection - total");
  #endif

  ROS_ASSERT(color.getHeight() == depth.getHeight());

  // -- Set up the message.
  bgmsg_.header.stamp.fromSec(wall_timestamp);
  bgmsg_.frame_id = frame_id;
  bgmsg_.sensor_timestamp = sensor_timestamp;
  bgmsg_.indices.clear();
  bgmsg_.depth.clear();
  bgmsg_.color.clear();
  int num = model_->widthStep() * model_->heightStep();
  bgmsg_.indices.resize(num);
  bgmsg_.depth.resize(num);
  bgmsg_.color.resize(num * 3);  // RGB    
  
  // -- Fill the message with data.
  uint8_t* color_data = (uint8_t*)color.getData();
  uint16_t* depth_data = (uint16_t*)depth.getData();
  size_t idx = 0;
  ROS_ASSERT(bg_index_y_ + fgmsg_.height_step <= bgmsg_.height);
  ROS_ASSERT(bg_index_x_ + fgmsg_.width_step <= bgmsg_.width);
  for(int y = bg_index_y_; y < bg_index_y_ + fgmsg_.height_step; ++y) {
    for(int x = bg_index_x_; x < bg_index_x_ + fgmsg_.width_step; ++x) {
      int i = y * bgmsg_.width + x;
      bgmsg_.indices[idx] = i;
      bgmsg_.depth[idx] = depth_data[i];
      bgmsg_.color[idx*3+0] = color_data[i*3+0];
      bgmsg_.color[idx*3+1] = color_data[i*3+1];
      bgmsg_.color[idx*3+2] = color_data[i*3+2];
      ++idx;
    }
  }

  // -- Advance to the next background index point.
  bg_index_x_ += fgmsg_.width_step;
  if(bg_index_x_ >= bgmsg_.width) {
    bg_index_x_ = 0;
    bg_index_y_ += fgmsg_.height_step;
    if(bg_index_y_ >= bgmsg_.height)
      bg_index_y_ = 0;
  }

  bg_pub_.publish(bgmsg_);
}

void ROSStreamingSentinel::handleDetection(openni::VideoFrameRef color,
                                           openni::VideoFrameRef depth,
                                           const std::vector<uint8_t>& mask,
                                           size_t num_in_mask,
                                           double sensor_timestamp,
                                           double wall_timestamp,
                                           size_t frame_id)
{

  #if JARVIS_DEBUG
  cout << "Entering ROSStreamingSentinel::handleDetection." << endl;
  size_t num = 0;
  for(size_t i = 0; i < mask.size(); ++i)
    if(mask[i] == 255 || mask[i] == 127)
      ++num;
  ROS_ASSERT(num == num_in_mask);
  #endif
  
  #if JARVIS_DEBUG
  ScopedTimer st("ROSStreamingSentinel::handleDetection - total");
  #endif

  if(!ros::ok()) {
    oni_.terminate();
    return;
  }

  ROS_ASSERT(color.getHeight() == depth.getHeight());

  fgmsg_.header.stamp.fromSec(wall_timestamp);
  fgmsg_.sensor_timestamp = sensor_timestamp;
  fgmsg_.frame_id = frame_id;
  fgmsg_.indices.clear();
  fgmsg_.depth.clear();
  fgmsg_.color.clear();
  fgmsg_.indices.resize(num_in_mask);
  fgmsg_.depth.resize(num_in_mask);
  fgmsg_.color.resize(num_in_mask * 3);  // RGB    

  uint8_t* color_data = (uint8_t*)color.getData();
  uint16_t* depth_data = (uint16_t*)depth.getData();
  size_t idx = 0;
  for(size_t i = 0; i < mask.size(); ++i) {
    if(mask[i] == 255 || mask[i] == 127) {
      fgmsg_.indices[idx] = i;
      fgmsg_.depth[idx] = depth_data[i];
      fgmsg_.color[idx*3+0] = color_data[i*3+0];
      fgmsg_.color[idx*3+1] = color_data[i*3+1];
      fgmsg_.color[idx*3+2] = color_data[i*3+2];
      ++idx;
    }
  }

  fgmsg_.fg_indices.clear();
  fgmsg_.bg_fringe_indices.clear();
  for(int y = fgmsg_.height_step / 2; y < fgmsg_.height; y += fgmsg_.height_step) {
    for(int x = fgmsg_.width_step / 2; x < fgmsg_.width; x += fgmsg_.width_step) {
      int idx = y * fgmsg_.width + x;
      if(mask[idx] == 255)
        fgmsg_.fg_indices.push_back(idx);
      else if(mask[idx] == 127)
        fgmsg_.bg_fringe_indices.push_back(idx);
    }
  }

  fg_pub_.publish(fgmsg_);
  
  #if JARVIS_DEBUG
  cout << "Leaving ROSStreamingSentinel::handleDetection." << endl;
  #endif
}



