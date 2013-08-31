#include <openni2_interface/openni_helpers.h>
#include <ros/assert.h>
#include <timer/timer.h>

void oniToCV(const openni::VideoFrameRef& oni, cv::Mat3b img)
{
  #if JARVIS_DEBUG
  ScopedTimer st("oniToCV");
  ROS_ASSERT(oni.getVideoMode().getPixelFormat() == openni::PIXEL_FORMAT_RGB888);
  ROS_ASSERT(oni.getWidth() == img.cols && oni.getHeight() == img.rows);
  #endif
  
  uchar* data = (uchar*)oni.getData();
  int i = 0;
  for(int y = 0; y < img.rows; ++y) {
    for(int x = img.cols - 1; x >= 0; --x, i+=3) {
      img(y, x)[0] = data[i+2];
      img(y, x)[1] = data[i+1];
      img(y, x)[2] = data[i];
    }
  }
}

cv::Mat3b oniToCV(const openni::VideoFrameRef& oni)
{
  cv::Mat3b cv(cv::Size(oni.getWidth(), oni.getHeight()));
  oniToCV(oni, cv);
  return cv;
}

DepthMatPtr oniDepthToEigenPtr(const openni::VideoFrameRef& oni)
{
  #if JARVIS_DEBUG
  ScopedTimer st("oniDepthToEigenPtr");
  #endif
  
  ROS_ASSERT(oni.getVideoMode().getPixelFormat() == openni::PIXEL_FORMAT_DEPTH_1_MM);

  DepthMatPtr depth(new DepthMat(oni.getHeight(), oni.getWidth()));
  ushort* data = (ushort*)oni.getData();
  int i = 0;
  for(int y = 0; y < depth->rows(); ++y)
    for(int x = depth->cols() - 1; x >= 0; --x, ++i)
      depth->coeffRef(y,x) = data[i];

  return depth;
}
  
DepthMat oniDepthToEigen(const openni::VideoFrameRef& oni)
{
  return *oniDepthToEigenPtr(oni);
}

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

