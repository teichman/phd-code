#include <rgbd_sequence/eigen_image.h>

using namespace Eigen;

EigenImage::EigenImage()
{
}

void EigenImage::resize(int width, int height)
{
  red_.resize(width, height);
  green_.resize(width, height);
  blue_.resize(width, height);
}

void EigenImage::setData(cv::Mat3b img)
{
  if(rows() != img.rows || cols() != img.cols)
    resize(img.rows, img.cols);
  
  for(int y = 0; y < img.rows; ++y) {
    for(int x = 0; x < img.cols; ++x) {
      blue_(y, x) = img(y, x)[0];
      green_(y, x) = img(y, x)[1];
      red_(y, x) = img(y, x)[2];
    }
  }
}

void EigenImage::fillCVImage(cv::Mat3b img) const
{
  ROS_ASSERT(img.rows == red_.rows());
  ROS_ASSERT(img.cols == red_.cols());

  for(int y = 0; y < img.rows; ++y) {
    for(int x = 0; x < img.cols; ++x) {
      img(y, x)[0] = blue_(y, x);
      img(y, x)[1] = green_(y, x);
      img(y, x)[2] = red_(y, x);
    }
  }
}

void EigenImage::save(const std::string& filename) const
{
  ogzstream file(filename.c_str());
  assert(file);
  serialize(file);
  file.close();
}

void EigenImage::load(const std::string& filename)
{
  igzstream file(filename.c_str());
  assert(file);
  deserialize(file);
  file.close();
}

void EigenImage::serialize(std::ostream& out) const
{
  eigen_extensions::serialize(red_, out);
  eigen_extensions::serialize(green_, out);
  eigen_extensions::serialize(blue_, out);
}

void EigenImage::deserialize(std::istream& in)
{
  eigen_extensions::deserialize(in, &red_);
  eigen_extensions::deserialize(in, &green_);
  eigen_extensions::deserialize(in, &blue_);
}

