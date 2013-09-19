#include <sentinel/reconstructor.h>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;

void Reconstructor::update(sentinel::ForegroundConstPtr fgmsg,
                           sentinel::BackgroundConstPtr bgmsg)
{
  // ----------------------------------------
  // -- Fancy way of doing this.
  // ----------------------------------------
  ROS_ASSERT(fgmsg->width == bgmsg->width);
  ROS_ASSERT(fgmsg->height == bgmsg->height);
  ROS_ASSERT(fgmsg->frame_id == bgmsg->frame_id);
  ROS_ASSERT(bgmsg->indices.size() == bgmsg->depth.size());
  
  for(size_t i = 1; i < bgmsg->indices.size(); ++i)
    ROS_ASSERT(bgmsg->indices[i-1] < bgmsg->indices[i]);
  vector<uint32_t> fg_indices = fgmsg->indices;
  sort(fg_indices.begin(), fg_indices.end());  

  size_t j = 0;
  for(size_t i = 0; i < bgmsg->indices.size(); ++i) {
    if(bgmsg->depth[i] == 0)
      continue;
    
    size_t idx = bgmsg->indices[i];
    ROS_ASSERT(idx < (size_t)img_.rows * img_.cols);
    
    // -- Skip pixels that are being sent as part of the Foreground message.
    //    They probably contain data that could corrupt the background view.
    while(j < fg_indices.size() && fg_indices[j] < idx)
      ++j;
    if(idx == fg_indices[j])
      continue;
    
    img_(idx)[0] = bgmsg->color[i*3+2];
    img_(idx)[1] = bgmsg->color[i*3+1];
    img_(idx)[2] = bgmsg->color[i*3+0];
  }
}

void Reconstructor::update(sentinel::ForegroundConstPtr fgmsg)
{
  // if(img_.rows != fgmsg->height || img_.cols != fgmsg->width)
  //   img_ = cv::Mat3b(cv::Size(fgmsg->width, fgmsg->height), cv::Vec3b(0, 0, 0));

  // sync_.addT0(fgmsg, fgmsg->sensor_timestamp);
  // process();
}

void Reconstructor::update(sentinel::BackgroundConstPtr bgmsg)
{
  if(img_.rows != bgmsg->height || img_.cols != bgmsg->width)
    img_ = cv::Mat3b(cv::Size(bgmsg->width, bgmsg->height), cv::Vec3b(0, 0, 0));

  for(size_t i = 0; i < bgmsg->indices.size(); ++i) {
    size_t idx = bgmsg->indices[i];
    ROS_ASSERT(idx < (size_t)img_.rows * img_.cols);
    img_(idx)[0] = bgmsg->color[i*3+2];
    img_(idx)[1] = bgmsg->color[i*3+1];
    img_(idx)[2] = bgmsg->color[i*3+0];
  }
  
  // sync_.addT1(bgmsg, bgmsg->sensor_timestamp);
  // process();
}

void Reconstructor::process()
{
  if(sync_.updated_) {
    update(sync_.current0_, sync_.current1_);
    sync_.updated_ = false;
  }
}

cv::Mat3b Reconstructor::stylizedImage(int scale) const
{
  // -- Get a black and white copy.
  cv::Mat3b vis;
  if(img_.rows == 0)
    return vis;
  cv::resize(img_, vis, img_.size() * scale, cv::INTER_NEAREST);
  for(int i = 0; i < vis.rows * vis.cols; ++i) {
    uint8_t val = ((float)vis(i)[0] + (float)vis(i)[1] + (float)vis(i)[2]) / 3.0;
    vis(i) = cv::Vec3b(val, val, val);
  }

  // -- Blur it out a bit.
  cv::GaussianBlur(vis, vis, cv::Size(11, 11), 1);

  return vis;
}


