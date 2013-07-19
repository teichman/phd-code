#ifndef ORB_EXTRACTOR_H
#define ORB_EXTRACTOR_H

#include <boost/shared_ptr.hpp>
#include <Eigen/Eigen>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/console.h>
#include <bag_of_tricks/high_res_timer.h>
#include <pipeline/pod.h>

/** \brief @b OrbExtractor produces nicely-formatted Orb features and descriptors.
 *   It is a wrapper around the OpenCV functionality.
 */
class OrbExtractor : public pl::Pod
{
public:  
  typedef Eigen::Matrix<uchar, 32, Eigen::Dynamic> PackedDescriptors;
  typedef boost::shared_ptr<PackedDescriptors> PackedDescriptorsPtr;
  typedef boost::shared_ptr<const PackedDescriptors> PackedDescriptorsConstPtr;
  typedef Eigen::Matrix<uchar, 32, 1> PackedDescriptor;

  DECLARE_POD(OrbExtractor);
  OrbExtractor(std::string name) :
    Pod(name)
  {
    declareParam<int>("DesiredNumKeypoints", 1000);
    declareParam<double>("ScaleFactor", 1.2);
    declareParam<int>("NumLevels", 3);
    declareParam<int>("FirstLevel", 0);
    
    declareInput<cv::Mat3b>("Image");
    declareOutput<PackedDescriptorsConstPtr>("Descriptors");
    declareOutput<const std::vector<cv::KeyPoint>*>("Keypoints");
  }

  void compute();
  void debug() const;
  //! Returns NULL if there are no keypoints.
  PackedDescriptorsPtr extractOrb(cv::Mat3b img, std::vector<cv::KeyPoint>* keypoints);

protected:
  boost::shared_ptr<cv::ORB> extractor_;
  std::vector<cv::KeyPoint> keypoints_;
    
};

#endif // ORB_EXTRACTOR_H
