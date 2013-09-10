#include <openni2_interface/openni2_interface.h>
#include <openni2_interface/openni_helpers.h>
#include <ros/assert.h>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace Eigen;

class Collector : public OpenNI2Handler
{
public:
  OpenNI2Interface oni_;
  size_t num_;

  Collector(size_t num, OpenNI2Interface::Resolution res = OpenNI2Interface::VGA) :
    oni_(res, res), num_(num)
  {
  }

  void collect()
  {
    oni_.setHandler(this);
    oni_.run();
  }

  void rgbdCallback(openni::VideoFrameRef color,
                    openni::VideoFrameRef depth,
                    size_t frame_id, double timestamp)
  {
    depth_images_.push_back(oniDepthToEigenPtr(depth));
    if(depth_images_.size() == num_)
      oni_.terminate();
  }

  void writeStats(std::string path) const;
  
protected:
  std::vector<DepthMatPtr> depth_images_;
  
};

void Collector::writeStats(std::string path) const
{
  ofstream f;
  f.open(path.c_str());
  ROS_ASSERT(f.is_open());

  ROS_ASSERT(depth_images_.size() > 0);
  
  ArrayXXd mean = ArrayXXd::Zero(depth_images_[0]->rows(), depth_images_[0]->cols());
  ArrayXXd count = ArrayXXd::Zero(mean.rows(), mean.cols());
  for(size_t i = 0; i < depth_images_.size(); ++i) {
    const DepthMat& depth = *depth_images_[i];
    for(int y = 0; y < depth.rows(); ++y) {
      for(int x = 0; x < depth.cols(); ++x) {
        if(depth(y, x) != 0) {
          mean(y, x) += depth(y, x) * 0.001;
          ++count(y, x);
        }
      }
    }
  }
  mean /= count;

  ArrayXXd var = ArrayXXd::Zero(mean.rows(), mean.cols());
  for(size_t i = 0; i < depth_images_.size(); ++i) {
    const DepthMat& depth = *depth_images_[i];
    for(int y = 0; y < depth.rows(); ++y)
      for(int x = 0; x < depth.cols(); ++x)
        var(y, x) += pow((depth(y, x) * 0.001 - mean(y, x)), 2);
  }
  var /= count;

  ArrayXXd stdev = ArrayXXd::Zero(mean.rows(), mean.cols());
  for(int y = 0; y < stdev.rows(); ++y)
    for(int x = 0; x < stdev.cols(); ++x)
      stdev(y, x) = sqrt(var(y, x));

  // stdev greater than 20cm is probably noise from flickering pixels.
  for(int y = 0; y < stdev.rows(); ++y)
    for(int x = 0; x < stdev.cols(); ++x)
      if(count(y, x) > depth_images_.size() / 2)
        if(stdev(y, x) < 0.2)
          f << mean(y, x) << " " << stdev(y, x) << endl;

  cv::Mat3b vis(cv::Size(stdev.cols(), stdev.rows()), cv::Vec3b(0, 0, 0));
  for(int y = 0; y < stdev.rows(); ++y)
    for(int x = 0; x < stdev.cols(); ++x)
      vis(y, x)[2] = min(1.0, stdev(y, x)) * 255;

  cv::imshow("vis", vis);
  cv::waitKey();
  
  f.close();
}


int main(int argc, char** argv)
{
  Collector col(100);
  col.collect();
  col.writeStats(argv[1]);

  return 0;
}
