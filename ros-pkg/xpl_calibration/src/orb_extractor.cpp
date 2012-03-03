#include <xpl_calibration/orb_extractor.h>

using namespace std;

void OrbExtractor::compute()
{
  cv::Mat3b img = pull<cv::Mat3b>("Image");
  PackedDescriptorsConstPtr descriptors = extractOrb(img, &keypoints_);

  push("Descriptors", descriptors);
  push<const vector<cv::Keypoints>*>("Keypoints", &keypoints_);
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
    extractor_ = shared_ptr<cv::Orb>(new cv::Orb(num, ocp));
  }
  
  // -- Compute keypoints on the image.
  cv::Mat descriptors; // descriptors is num_descr x 32
  extractor_(img, cv::Mat(), *keypoints, descriptors);
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

OrbExtractor::debug()
{

}
