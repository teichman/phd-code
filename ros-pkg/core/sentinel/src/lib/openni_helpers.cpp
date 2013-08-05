#include <sentinel/openni_helpers.h>

DepthMatPtr oniDepthToEigenPtr(const openni_wrapper::DepthImage& oni)
{
  DepthMatPtr depth(new DepthMat(oni.getHeight(), oni.getWidth()));
  unsigned short data[depth->rows() * depth->cols()];
  oni.fillDepthImageRaw(depth->cols(), depth->rows(), data);
  int i = 0;
  for(int y = 0; y < depth->rows(); ++y){
    for(int x = 0; x < depth->cols(); ++x, ++i){
      if(data[i] == oni.getNoSampleValue() || data[i] == oni.getShadowValue()){
        depth->coeffRef(y,x) = 0;
      }else{
        depth->coeffRef(y,x) = data[i];
      }
    }
  }
  return depth;
}
  
DepthMat oniDepthToEigen(const openni_wrapper::DepthImage& oni)
{
  return *oniDepthToEigenPtr(oni);
}

cv::Mat3b oniToCV(const openni_wrapper::Image& oni)
{
  cv::Mat3b img(oni.getHeight(), oni.getWidth());
  uchar data[img.rows * img.cols * 3];
  oni.fillRGB(img.cols, img.rows, data);
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
