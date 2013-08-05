#ifndef OPENNI_HELPERS_H
#define OPENNI_HELPERS_H

#include <Eigen/Eigen>
#include <boost/shared_ptr.hpp>
#include <opencv2/core/core.hpp>
#include <pcl/io/openni_grabber.h>

typedef Eigen::Matrix<unsigned short, Eigen::Dynamic, Eigen::Dynamic> DepthMat;  
typedef boost::shared_ptr<DepthMat> DepthMatPtr;
typedef boost::shared_ptr<const DepthMat> DepthMatConstPtr;

DepthMat oniDepthToEigen(const openni_wrapper::DepthImage& oni);
DepthMatPtr oniDepthToEigenPtr(const openni_wrapper::DepthImage& oni);
//! Warning: Once upon a time this depended on having
//! the patched version of OpenNI that comes with PCL.
cv::Mat3b oniToCV(const openni_wrapper::Image& oni);

#endif // OPENNI_HELPERS_H
