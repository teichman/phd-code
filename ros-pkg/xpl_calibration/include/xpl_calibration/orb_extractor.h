#ifndef ORB_EXTRACTOR_H
#define ORB_EXTRACTOR_H

#include <boost/shared_ptr.hpp>
#include <Eigen/Eigen>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <serializable/serializable.h>

/** \brief @b OrbExtractor produces nicely-formatted Orb features and descriptors.
 *   It is a wrapper around the OpenCV functionality.
 */
class OrbExtractor
{
public:
  typedef Eigen::Matrix<uchar, 32, Eigen::Dynamic> PackedDescriptors;
  typedef boost::shared_ptr<PackedDescriptors> PackedDescriptorsPtr;
  typedef Eigen::Matrix<uchar, 32, 1> PackedDescriptor;

  cv::ORB extractor_;

  OrbExtractor();
  //! Returns NULL if there are no keypoints.
  PackedDescriptorsPtr extractOrb(cv::Mat3b img, std::vector<cv::KeyPoint>* keypoints);
};

#endif // ORB_EXTRACTOR_H
