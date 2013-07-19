#ifndef GAUSSIAN_BACKGROUND_MODELER_H
#define GAUSSIAN_BACKGROUND_MODELER_H

#include <xpl_calibration/common.h>

class GaussianBackgroundModeler : public pl::Pod
{
public:
  DECLARE_POD(GaussianBackgroundModeler);
  GaussianBackgroundModeler(std::string name) :
    Pod(name)
  {
    declareParam<int>("Ignore", 10); // Ignore every k frames for each one used.
    declareParam<double>("Stdevs", 3.0); // How many stdevs outside of mean is still considered background.
    declareInput<rgbd::Sequence::ConstPtr>("Sequence");
    // Points between min and max are background.
    declareOutput<const std::vector<double>*>("MinDistances");
    declareOutput<const std::vector<double>*>("MaxDistances");
  }

  void compute();
  void debug() const;

protected:
  std::vector<double> min_distances_;
  std::vector<double> max_distances_;
  std::vector<double> stdevs_;
  std::vector<double> means_;
  std::vector<double> counts_;
  
  cv::Mat1f getZBuffer(const rgbd::Cloud& pcd) const;
  cv::Mat1b visualizeVector(const std::vector<double>& vec) const;
};

#endif // GAUSSIAN_BACKGROUND_MODELER_H
