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
  if(depth_res == OpenNI2Interface::VGA) {
    model_ = boost::shared_ptr<BackgroundModel>(new BackgroundModel(640, 480, 16, 12, 0.1, MAX_DEPTH, 0.2));
  }
  else if(depth_res == OpenNI2Interface::QVGA)
    model_ = boost::shared_ptr<BackgroundModel>(new BackgroundModel(320, 240, 8, 6, 0.1, MAX_DEPTH, 0.2));
  else {
    ROS_ASSERT(0);
  }

  indices_.reserve(model_->width() * model_->height());
  size_t max_num_markers = (model_->width() / model_->widthStep())
    * (model_->height() / model_->heightStep());
  fg_markers_.reserve(max_num_markers);
  bg_fringe_markers_.reserve(max_num_markers);
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
  cout << "############################################################" << endl;
  #endif
  ScopedTimer st("Sentinel::rgbdCallback");

  // -- Check for misaligned data.
  //    TODO: This should probably be in OpenNI2Interface.
  double depth_timestamp = (double)oni_depth.getTimestamp() * 1e-6;
  #if JARVIS_DEBUG
  double image_timestamp = (double)oni_color.getTimestamp() * 1e-6;
  double thresh = 1.1 * (1.0 / 60.0);
  if(fabs(depth_timestamp - image_timestamp) > thresh) {
    ROS_WARN_STREAM("rgbdCallback got an rgbd pair with timestamp delta of "
                    << depth_timestamp - image_timestamp);
  }
  #endif

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
  
  // -- If the model has been trained suffificiently, make predictions.
  if((int)training_.size() == 0)
    return;
  
  // -- Get raw mask.
  {
    #if JARVIS_DEBUG
    ScopedTimer st("Making predictions");
    #endif
    
    model_->predict(depth, &indices_, &fg_markers_, &bg_fringe_markers_);
  }

  // -- Process the detection.
  if((double)fg_markers_.size() / model_->size() > threshold_) {
    handleDetection(color, depth, indices_, fg_markers_, bg_fringe_markers_,
                    sensor_timestamp, wall_timestamp, frame_id);
  }
  handleNonDetection(color, depth, sensor_timestamp, wall_timestamp, frame_id);
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
  fg_pub_ = nh_.advertise<sentinel::Foreground>("foreground", 0);
  bg_pub_ = nh_.advertise<sentinel::Background>("background", 0);
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
  ScopedTimer st("ROSStreamingSentinel::handleNonDetection");
  #endif

  ROS_ASSERT(color.getHeight() == depth.getHeight());

  // -- Set up the message.
  bgmsg_.header.stamp.fromSec(wall_timestamp);
  bgmsg_.frame_id = frame_id;
  bgmsg_.sensor_timestamp = sensor_timestamp;
  bgmsg_.color_pixel_type = color.getVideoMode().getPixelFormat();
  bgmsg_.depth_pixel_type = depth.getVideoMode().getPixelFormat();
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
                                           const std::vector<uint32_t>& indices,
                                           const std::vector<uint32_t>& fg_markers,
                                           const std::vector<uint32_t>& bg_fringe_markers,
                                           double sensor_timestamp,
                                           double wall_timestamp,
                                           size_t frame_id)
{  
  #if JARVIS_DEBUG
  ScopedTimer st("ROSStreamingSentinel::handleDetection");
  #endif

  if(!ros::ok()) {
    oni_.terminate();
    return;
  }

  ROS_ASSERT(color.getHeight() == depth.getHeight());

  fgmsg_.header.stamp.fromSec(wall_timestamp);
  fgmsg_.frame_id = frame_id;
  fgmsg_.sensor_timestamp = sensor_timestamp;
  fgmsg_.color_pixel_type = color.getVideoMode().getPixelFormat();
  fgmsg_.depth_pixel_type = depth.getVideoMode().getPixelFormat();

  fgmsg_.indices = indices;
  fgmsg_.fg_indices = fg_markers;
  fgmsg_.bg_fringe_indices = bg_fringe_markers;

  fgmsg_.depth.resize(indices.size());
  fgmsg_.color.resize(indices.size() * 3);  // RGB    
  uint8_t* color_data = (uint8_t*)color.getData();
  uint16_t* depth_data = (uint16_t*)depth.getData();
  size_t idx = 0;
  for(size_t i = 0; i < indices.size(); ++i) {
    size_t idx = indices[i];
    fgmsg_.depth[i] = depth_data[idx];
    fgmsg_.color[i*3+0] = color_data[idx*3+0];
    fgmsg_.color[i*3+1] = color_data[idx*3+1];
    fgmsg_.color[i*3+2] = color_data[idx*3+2];
  }

  fg_pub_.publish(fgmsg_);
}



