#ifndef OBJECT_EXTRACTOR_H
#define OBJECT_EXTRACTOR_H

#include <xpl_calibration/common.h>

class ObjectExtractor : public pipeline::Pod
{
public:
  typedef std::vector< std::vector<rgbd::Cloud::ConstPtr> > Objects;
  
  DECLARE_POD(ObjectExtractor);
  ObjectExtractor(std::string name) :
    Pod(name)
  {
    declareParam<int>("MinClusterSize", 100);
    declareParam<double>("ClusterTolerance", 0.4); // meters
    
    declareInput<rgbd::Sequence::ConstPtr>("Sequence");
    declareInput<const std::vector<cv::Mat1b>*>("ForegroundImages");
    declareInput<const std::vector< std::vector<int> >*>("ForegroundIndices");
    
    declareOutput<const Objects*>("Objects");
  }

  void compute();

protected:
  Objects objects_;

  void extractObjectsFromFrame(const rgbd::Cloud& pcd,
			       const std::vector<int>& indices,
			       std::vector<rgbd::Cloud::ConstPtr>* objects) const;
};

#endif // OBJECT_EXTRACTOR_H
