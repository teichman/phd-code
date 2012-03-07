#ifndef OBJECT_EXTRACTOR_H
#define OBJECT_EXTRACTOR_H

#include <xpl_calibration/common.h>

class ObjectExtractor : public pipeline::Pod
{
public:
  typedef std::vector< std::vector<rgbd::Cloud::ConstPtr> > Objects;
  typedef std::vector< std::vector< std::vector<int> > > ObjectIndices;
  
  DECLARE_POD(ObjectExtractor);
  ObjectExtractor(std::string name) :
    Pod(name)
  {
    declareParam<double>("MinClusterSize", 0.5);
    declareParam<int>("MinClusterPoints", 1000);
    declareParam<double>("ClusterTolerance", 0.1); // meters
    
    declareInput<rgbd::Sequence::ConstPtr>("Sequence");
    declareInput<const std::vector<cv::Mat1b>*>("ForegroundImages");
    declareInput<const std::vector< std::vector<int> >*>("ForegroundIndices");
    
    declareOutput<const Objects*>("Objects");
    declareOutput<const ObjectIndices*>("ObjectIndices"); // Indexes into the original cloud, not foreground indices.
  }

  void compute();
  void debug() const;

protected:
  Objects objects_;
  ObjectIndices object_indices_;

  void extractObjectsFromFrame(const rgbd::Cloud& pcd,
			       const std::vector<int>& indices,
			       std::vector<rgbd::Cloud::ConstPtr>* objects,
			       std::vector< std::vector<int> >* object_indices) const;
};

#endif // OBJECT_EXTRACTOR_H
