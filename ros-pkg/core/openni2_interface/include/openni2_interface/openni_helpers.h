#ifndef OPENNI_HELPERS_H
#define OPENNI_HELPERS_H

#include <Eigen/Eigen>
#include <boost/shared_ptr.hpp>
#include <opencv2/core/core.hpp>
#include <OpenNI.h>

typedef Eigen::Matrix<unsigned short, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> DepthMat;  
typedef boost::shared_ptr<DepthMat> DepthMatPtr;
typedef boost::shared_ptr<const DepthMat> DepthMatConstPtr;

void oniToCV(const openni::VideoFrameRef& color, cv::Mat3b img);
cv::Mat3b oniToCV(const openni::VideoFrameRef& oni);
DepthMat oniDepthToEigen(const openni::VideoFrameRef& depth);
DepthMatPtr oniDepthToEigenPtr(const openni::VideoFrameRef& depth);
cv::Vec3b colorize(double depth, double min_range, double max_range);
  
#endif // OPENNI_HELPERS_H