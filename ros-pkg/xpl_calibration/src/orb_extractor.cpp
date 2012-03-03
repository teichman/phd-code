#include <xpl_calibration/orb_extractor.h>

using namespace std;

void OrbExtractor::compute()
{
  cv::Mat3b img = pull<cv::Mat3b>("Image");
  PackedDescriptorsConstPtr descriptors = extractOrb(img, &keypoints_);

  push("Descriptors", descriptors);
  push<const vector<cv::KeyPoint>*>("Keypoints", &keypoints_);
}

OrbExtractor::PackedDescriptorsPtr OrbExtractor::extractOrb(cv::Mat3b img,
							    vector<cv::KeyPoint>* keypoints)
{
  keypoints_.clear();
  if(!extractor_) {
    cv::ORB::CommonParams ocp(param<double>("ScaleFactor"),
			      param<int>("NumLevels"),
			      31,
			      param<int>("FirstLevel"));
    int num = param<int>("DesiredNumKeypoints");
    extractor_ = boost::shared_ptr<cv::ORB>(new cv::ORB(num, ocp));
  }
  
  // -- Compute keypoints on the image.
  cv::Mat descriptors; // descriptors is num_descr x 32
  (*extractor_)(img, cv::Mat(), *keypoints, descriptors);
  assert(descriptors.type() == CV_8UC1);
  
  if(keypoints->empty())
    return PackedDescriptorsPtr((PackedDescriptors*)NULL);

  // -- Build the descriptors matrix for the filtered keypoints.
  PackedDescriptorsPtr packed(new PackedDescriptors(32, descriptors.rows));
  for(int i = 0; i < packed->cols(); ++i) { 
    for(int j = 0; j < packed->rows(); ++j) {
      packed->coeffRef(j, i) = descriptors.at<uchar>(i, j);
    }
  }

  return packed;
}

void OrbExtractor::debug() const
{
  cv::Mat3b vis = pull<cv::Mat3b>("Image").clone();
  for(size_t i = 0; i < keypoints_.size(); ++i)
    cv::circle(vis, keypoints_[i].pt, 2, cv::Scalar(0, 0, 255), -1);

  cv::imwrite(getDebugPath() + "-keypoints.png", vis);
}
