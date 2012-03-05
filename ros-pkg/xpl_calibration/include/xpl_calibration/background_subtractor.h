#ifndef BACKGROUND_SUBTRACTOR_H
#define BACKGROUND_SUBTRACTOR_H

#include <xpl_calibration/background_modeler.h>

class BackgroundSubtractor : public pipeline::Pod
{
public:
  DECLARE_POD(BackgroundSubtractor);
  BackgroundSubtractor(std::string name) :
    Pod(name)
  {
    declareInput<const BackgroundModel*>("BackgroundModel");
    declareInput<rgbd::Sequence::ConstPtr>("Sequence");
    declareParam<int>("NumErosions", 5);
    declareOutput<const std::vector< std::vector<int> >*>("ForegroundIndices");
  }

  void compute();
  void debug() const;

protected:
  std::vector< std::vector<int> > fg_indices_;

  void findForeground(const rgbd::Cloud& pcd,
		      const BackgroundModel& model,
		      std::vector<int>* indices) const;
    
};

#endif // BACKGROUND_SUBTRACTOR_H
