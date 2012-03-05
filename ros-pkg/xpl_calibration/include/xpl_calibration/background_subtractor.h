#ifndef BACKGROUND_SUBTRACTOR_H
#define BACKGROUND_SUBTRACTOR_H

#include <xpl_calibration/common.h>

class BackgroundSubtractor : public pipeline::Pod
{
public:
  DECLARE_POD(BackgroundSubtractor);
  BackgroundSubtractor(std::string name) :
    Pod(name)
  {
    // Min and max distances define the range that is considered background for each pixel.
    declareInput<const std::vector<double>*>("MinDistances");
    declareInput<const std::vector<double>*>("MaxDistances");
    declareInput<rgbd::Sequence::ConstPtr>("Sequence");

    declareOutput<const std::vector< std::vector<int> >*>("ForegroundIndices");
  }

  void compute();
  void debug() const;

protected:
  std::vector< std::vector<int> > fg_indices_;

  void findForeground(const rgbd::Cloud& pcd,
		      const std::vector<double>& min_distances,
		      const std::vector<double>& max_distances,
		      std::vector<int>* indices) const;
    
};

#endif // BACKGROUND_SUBTRACTOR_H
