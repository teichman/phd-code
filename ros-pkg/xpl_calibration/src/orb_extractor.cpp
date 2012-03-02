#include <xpl_calibration/orb_extractor.h>

using namespace std;

OrbExtractor::OrbExtractor() :
  extractor_(1000, cv::ORB::CommonParams(1.2, 3, 7, 0))
{
}

OrbExtractor::PackedDescriptorsPtr OrbExtractor::extractOrb(cv::Mat3b img,
							    vector<cv::KeyPoint>* keypoints)
{
  // -- Compute keypoints on the image.
  cv::Mat descriptors; // descriptors is num_descr x 32

  // Strangely, OpenCV is not computing orb features anywhere except the very middle of the image.
  // Workaround: pad the image.
  // cv::Mat3b buf(img.size() * 3, cv::Vec3b(0, 0, 0));
  // for(int y = 0; y < buf.rows; ++y) {
  //   for(int x = 0; x < buf.cols; ++x) {
  //     if(x >= img.cols && x < 2*img.cols &&
  // 	 y >= img.rows && y < 2*img.rows)
  // 	buf(y, x) = img(y - img.rows, x - img.cols);
  //   }
  // }
  //extractor_(buf, cv::Mat(), *keypoints, descriptors);

  // // Translate back to original image coords.
  // for(size_t i = 0; i < keypoints->size(); ++i) {
  //   keypoints->at(i).pt.x -= img.cols;
  //   keypoints->at(i).pt.y -= img.rows;
  // }

  extractor_(img, cv::Mat(), *keypoints, descriptors);
  // cout << "cols: " << descriptors.cols << endl;
  // cout << "rows: " << descriptors.rows << endl;
  // cout << descriptors.type() << endl;
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
