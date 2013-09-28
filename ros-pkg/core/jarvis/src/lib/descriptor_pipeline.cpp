#include <pipeline/outlet.h>
#include <pipeline/common_pods.h>
#include <ros/package.h>
#include <jarvis/descriptor_pipeline.h>

using namespace std;
using namespace Eigen;
using namespace pl;

DescriptorPipeline::DescriptorPipeline() :
  pl_(1)
{
  registerPodTypes();
}

void DescriptorPipeline::registerPodTypes()
{
  REGISTER_POD_TEMPLATE(EntryPoint, Blob::Ptr);
  REGISTER_POD(BlobProjector);
  REGISTER_POD(BoundingBoxSize);
  REGISTER_POD(DescriptorAggregator);
}

std::string DescriptorPipeline::defaultSpecificationPath()
{
  return ros::package::getPath("jarvis") + "/config/default_descriptor_pipeline.yml";
}

YAML::Node DescriptorPipeline::defaultSpecification()
{
  return YAML::LoadFile(defaultSpecificationPath())["Pipeline"];
}

void DescriptorPipeline::initializeWithDefault()
{
  initialize(defaultSpecification());
}

void DescriptorPipeline::initialize(YAML::Node spec)
{
  pl_.deYAMLize(spec);
}

const vector<const VectorXf*>* DescriptorPipeline::computeDescriptors(Blob::Ptr blob)
{
  pl_.push("BlobEntryPoint", blob);
  pl_.compute();
  const vector<const VectorXf*>* descriptors;
  pl_.pull("DescriptorAggregator.AggregatedDescriptors", &descriptors);
  return descriptors;
}


/************************************************************
 * Helper functions
 ************************************************************/

double updateDescriptors(YAML::Node plspec, int num_threads, TrackDataset* td)
{
  HighResTimer hrt;
  hrt.start();

  // -- Delete all the existing descriptors.
  //    Testing the hypothesis that this will fix memory fragmentation issues.
  td->deleteDescriptors();
  
  // -- Apply the new dmap.
  //    This will de-allocate any descriptors that are no longer needed,
  //    move those that need to be moved,
  //    and make a new NULL descriptor for spaces that should be filled.
  DescriptorPipeline dp;
  dp.initialize(plspec);
  td->applyNameMapping("dmap", dp.dmap());
  
  #pragma omp parallel for
  for(int tidx = 0; tidx < num_threads; ++tidx) {
    DescriptorPipeline dp;
    dp.initialize(plspec);
    
    for(size_t i = tidx; i < td->tracks_.size(); i += num_threads) {
      Dataset& track = (*td)[i];
      for(size_t j = 0; j < track.size(); ++j) {
        Instance& inst = track[j];
        Blob::Ptr blob = boost::any_cast<Blob::Ptr>(inst.raw_);

        // -- Run the pipeline.
        const vector<const VectorXf*>* descriptors = dp.computeDescriptors(blob);
        //cout << dp.reportTiming() << endl;

        // -- Fill the Instance with the new descriptors.
        ROS_ASSERT(inst.descriptors_.size() == descriptors->size());
        inst.copy(*descriptors);
        ROS_ASSERT(inst.descriptors_.size() == descriptors->size());
      }
    }
  }
  
  hrt.stop();
  double ms_per_obj = hrt.getMilliseconds() / td->totalInstances();
  cout << "updateDescriptors: " << ms_per_obj << " ms / obj." << endl;
  return ms_per_obj;
}
