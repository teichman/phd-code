#ifndef EIGEN_IMAGE_H
#define EIGEN_IMAGE_H

#include <opencv2/core/core.hpp>
#include <ros/assert.h>
#include <serializable/serializable.h>
#include <eigen_extensions/eigen_extensions.h>

//! Simple container for comparing with the speed and compression of saving pngs.
class EigenImage : public Serializable
{
public:
  typedef Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> Channel; 
  
  EigenImage();
  void resize(int width, int height);
  void setData(cv::Mat3b img);
  void fillCVImage(cv::Mat3b img) const;
  int rows() const { return red_.rows(); }
  int cols() const { return red_.cols(); }
  void save(const std::string& filename) const;
  void load(const std::string& filename);
  void serialize(std::ostream& out) const;
  void deserialize(std::istream& in);

protected:
  Channel red_;
  Channel green_;
  Channel blue_;
};

#endif // EIGEN_IMAGE_H
