#include <sentinel/openni_helpers.h>
#include <ros/assert.h>

cv::Mat3b oniToCV(const openni::VideoFrameRef& oni)
{
  ROS_ASSERT(oni.getVideoMode().getPixelFormat() == openni::PIXEL_FORMAT_RGB888);
  
  cv::Mat3b img(oni.getHeight(), oni.getWidth());
  uchar* data = (uchar*)oni.getData();
  int i = 0;
  for(int y = 0; y < img.rows; ++y) {
    for(int x = 0; x < img.cols; ++x, i+=3) {
      img(y, x)[0] = data[i+2];
      img(y, x)[1] = data[i+1];
      img(y, x)[2] = data[i];
    }
  }
    
  return img;
}

DepthMatPtr oniDepthToEigenPtr(const openni::VideoFrameRef& oni)
{
  ROS_ASSERT(oni.getVideoMode().getPixelFormat() == openni::PIXEL_FORMAT_DEPTH_1_MM);

  DepthMatPtr depth(new DepthMat(oni.getHeight(), oni.getWidth()));
  ushort* data = (ushort*)oni.getData();
  int i = 0;
  for(int y = 0; y < depth->rows(); ++y) {
    for(int x = 0; x < depth->cols(); ++x, ++i) {
      depth->coeffRef(y,x) = data[i];
    }
  }
  return depth;
}
  
DepthMat oniDepthToEigen(const openni::VideoFrameRef& oni)
{
  return *oniDepthToEigenPtr(oni);
}


