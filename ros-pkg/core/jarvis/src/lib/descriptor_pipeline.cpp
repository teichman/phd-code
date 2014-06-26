#include <jarvis/descriptor_pipeline.h>
#include <online_learning/instance_serializer.h>
#include <pipeline/outlet.h>
#include <pipeline/common_pods.h>
#include <ros/package.h>

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
  REGISTER_POD_TEMPLATE(EntryPoint, Blob::ConstPtr);
  REGISTER_POD(TrajectoryAccumulator);
  REGISTER_POD(TrajectoryStatistics);
  REGISTER_POD(SimpleTrajectoryStatistics);
  REGISTER_POD(BlobProjector);
  REGISTER_POD(BoundingBoxSize);
  REGISTER_POD(DescriptorAggregator);
  REGISTER_POD(CloudOrienter);
  REGISTER_POD(GravitationalCloudOrienter);
  REGISTER_POD(CloudSelector);
  REGISTER_POD(CentroidFinder);
  REGISTER_POD(NormalizedDensityHistogram);
  REGISTER_POD(CloudProjector);
  REGISTER_POD(DynamicImageWindow);
  REGISTER_POD(HogArray);
  REGISTER_POD(IntensityHistogram);
  REGISTER_POD(HSVHistogram);
  REGISTER_POD(RandomProjector);
  REGISTER_POD(DescriptorConcatenator);
  REGISTER_POD(EdginessEstimator);
  REGISTER_POD(ProjectedSize);
  REGISTER_POD(ThermalGrabberPod);
  REGISTER_POD(AverageTemperature);
}

std::string DescriptorPipeline::defaultSpecificationPath()
{
  return ros::package::getPath("jarvis") + "/config/config.yml";
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

  if(getenv("THERMAL_DATA")) {
    string path(getenv("THERMAL_DATA"));
    cout << "Initializing ThermalGrabberPod with data at path \"" << path << "\"" << endl;
    pl_.pod<ThermalGrabberPod>()->initializeThermalGrabber(path);
    cout << "Done." << endl;
  }
}

const vector<const VectorXf*>* DescriptorPipeline::computeDescriptors(Blob::ConstPtr blob)
{
  pl_.push("BlobEntryPoint", blob);
  pl_.compute();
  const vector<const VectorXf*>* descriptors;
  pl_.pull("DescriptorAggregator.AggregatedDescriptors", &descriptors);
  return descriptors;
}

void DescriptorPipeline::setUpVector(const Eigen::VectorXf up)
{
  vector<GravitationalCloudOrienter*> gcos = pl_.filterPods<GravitationalCloudOrienter>();
  for(size_t i = 0; i < gcos.size(); ++i)
    gcos[i]->setUpVector(up);
}


/************************************************************
 * Helper functions
 ************************************************************/

double updateDescriptors(YAML::Node plspec, int num_threads, TrackDataset* td, Eigen::VectorXf up, bool debug)
{
  HighResTimer hrt;
  hrt.start();

  DescriptorPipeline dp;
  dp.initialize(plspec);
  if(up.rows() == 3)
    dp.setUpVector(up);
  
  if(!debug && td->nameMapping("dmap") == dp.dmap()) {
    cout << "updateDescriptors: nothing to do" << endl;
    return 0;
  }
  
  // -- Delete all the existing descriptors.
  //    Testing the hypothesis that this will fix memory fragmentation issues.
  //    ... looks like it doesn't.  Hm.
  //    12Gb total memory usage -> 43Gb after doing an update... and that's only
  //    when adding 60 new axes to the total descriptor space (out of ~4000).
  td->deleteDescriptors();

  // -- Apply the new dmap.
  //    This will de-allocate any descriptors that are no longer needed,
  //    move those that need to be moved,
  //    and make a new NULL descriptor for spaces that should be filled.
  td->applyNameMapping("dmap", dp.dmap());
  
  #pragma omp parallel for
  for(int tidx = 0; tidx < num_threads; ++tidx) {
    DescriptorPipeline dp;
    dp.initialize(plspec);
    if(debug)
      dp.setDebug(debug);
    if(up.rows() == 3)
      dp.setUpVector(up);

    
    for(size_t i = tidx; i < td->tracks_.size(); i += num_threads) {
      Dataset& track = (*td)[i];
      for(size_t j = 0; j < track.size(); ++j) {
        Instance& inst = track[j];
        ROS_ASSERT(!inst.raw().empty());
        Blob::ConstPtr blob = boost::any_cast<Blob::ConstPtr>(inst.raw());

        // -- Run the pipeline.
        const vector<const VectorXf*>* descriptors = dp.computeDescriptors(blob);
        if(debug)
          cout << dp.reportTiming() << endl;

        // -- Fill the Instance with the new descriptors.
        ROS_ASSERT(inst.descriptors_.size() == descriptors->size());
        inst.copy(*descriptors);
        ROS_ASSERT(inst.descriptors_.size() == descriptors->size());

        // -- De-project the blobs. ....  but we need the PCD for visualization.  That's annoying.
        blob->cloud_.reset();
        blob->kdtree_.reset();
      }
      // Clear any data that accumulated over the course of the track.
      // This includes, for example, TrajectoryAccumulator.
      dp.reset();
    }
  }
  
  hrt.stop();
  double ms_per_obj = hrt.getMilliseconds() / td->totalInstances();
  cout << "updateDescriptors: " << ms_per_obj << " ms / obj." << endl;
  
  return ms_per_obj;
}

TrackDataset::Ptr loadDatasets(const std::vector<std::string> paths,
                               YAML::Node config,
                               const NameMapping& cmap,
                               const Eigen::VectorXf& up,
                               bool verbose)
{
  ROS_ASSERT(!paths.empty());

  ROS_DEBUG_STREAM("loadDatasets called with Instance::custom_serializer_ of "
                   << Instance::custom_serializer_->name());
      
  TrackDataset::Ptr data(new TrackDataset());
  data->load(paths[0]);
  updateDescriptors(config["Pipeline"], 24, data.get(), up);
  if(!cmap.empty())
    data->applyNameMapping("cmap", cmap);
  
  for(size_t i = 1; i < paths.size(); ++i) {
    if(verbose)
      cout << "Loading " << paths[i] << endl;
    TrackDataset tmp;
    tmp.load(paths[i]);
    updateDescriptors(config["Pipeline"], 24, &tmp, up);
    if(!cmap.empty())
      tmp.applyNameMapping("cmap", cmap);
    ROS_ASSERT(data->nameMappingsAreEqual(tmp));
    *data += tmp;
  }

  return data;
}

TrackDataset::Ptr loadDatasets(const std::vector<std::string> paths,
                               YAML::Node config,
                               const NameMapping& cmap,
                               bool verbose)
{
  ROS_ASSERT(!paths.empty());

  ROS_DEBUG_STREAM("loadDatasets called with Instance::custom_serializer_ of "
                   << Instance::custom_serializer_->name());
      
  TrackDataset::Ptr data(new TrackDataset());
  data->load(paths[0]);
  updateDescriptors(config["Pipeline"], 24, data.get());
  if(!cmap.empty())
    data->applyNameMapping("cmap", cmap);
  
  for(size_t i = 1; i < paths.size(); ++i) {
    if(verbose)
      cout << "Loading " << paths[i] << endl;
    TrackDataset tmp;
    tmp.load(paths[i]);
    updateDescriptors(config["Pipeline"], 24, &tmp);
    if(!cmap.empty())
      tmp.applyNameMapping("cmap", cmap);
    ROS_ASSERT(data->nameMappingsAreEqual(tmp));
    *data += tmp;
  }

  return data;
}

