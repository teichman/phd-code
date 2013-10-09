#include <online_learning/evaluator.h>
#include <jarvis/twiddler.h>
#include <jarvis/descriptor_pipeline.h>

using namespace std;
using namespace pl;

/************************************************************
 * JarvisTwiddler
 ************************************************************/

JarvisTwiddler::JarvisTwiddler(TrackDataset::Ptr train,
                               TrackDataset::Ptr test,
                               int num_threads) :
  PipelineTwiddler(),
  num_threads_(num_threads),
  train_(train),
  test_(test),
  speed_check_(new TrackDataset)
{
  // -- Shuffle the training and test set so we don't get bad
  //    core utilization due to datasets ordered by track length.
  random_shuffle(train_->tracks_.begin(), train_->tracks_.end());
  random_shuffle(test_->tracks_.begin(), test_->tracks_.end());
  
  // -- Set up speed check dataset.
  //    We need to deep copy here otherwise the name mapping
  //    would change for some Dataset objects but not
  //    for the containing TrackDataset.
  size_t sc_size = 50;
  ROS_ASSERT(train_->size() > sc_size);
  speed_check_->applyNameMappings(*train_);
  speed_check_->tracks_.resize(sc_size);
  for(size_t i = 0; i < speed_check_->size(); ++i)
    speed_check_->tracks_[i] = Dataset::Ptr(new Dataset(*train_->tracks_[i]));
  
  // -- Set up twiddle actions.
  REGISTER_ACTION(JarvisTwiddler::twiddleNumCells);
  REGISTER_ACTION(JarvisTwiddler::twiddleTrainerThreshold);
  REGISTER_ACTION(JarvisTwiddler::deleteRandomPod);
  REGISTER_ACTION(JarvisTwiddler::addRawNormalizedHistogramBranch);
  REGISTER_ACTION(JarvisTwiddler::addOrientedNormalizedHistogramBranch);
}

bool JarvisTwiddler::isRequired(Pod* pod)
{
  return (
    dynamic_cast< EntryPoint<Blob::Ptr>* >(pod) ||
    dynamic_cast<DescriptorAggregator*>(pod)
    );
}

void JarvisTwiddler::improvementHook(const YAML::Node& config,
                                     const YAML::Node& results,
                                     std::string evalpath) const
{
  Twiddler::improvementHook(config, results, evalpath);
  Pipeline pl(1);
  pl.deYAMLize(config["Pipeline"]);
  pl.writeGraphviz(root_dir_ + "/best_pipeline.gv");
}

YAML::Node JarvisTwiddler::evaluate(const YAML::Node& config, std::string evalpath)
{
  YAML::Node results;
  results["NoDescriptors"] = false;
  results["DescriptorComputationTooSlow"] = false;
  
  // -- Save the pipeline we are evaluating in graphviz format.
  Pipeline pl(1);
  pl.deYAMLize(config["Pipeline"]);
  pl.writeGraphviz(evalpath + "/pipeline.gv");
  if(pl.pod<DescriptorAggregator>()->dmap().size() == 0) {
    results["NoDescriptors"] = true;
    return results;
  }

  // Check memory usage.
  int retval = system(("free -m > " + evalpath + "/free.txt").c_str()); --retval;

  // -- Compute descriptors on training set.  If too slow, don't proceed.
  double ms_per_obj = updateDescriptors(config["Pipeline"], num_threads_, train_.get());
  results["MsPerObj"] = ms_per_obj;
  if(ms_per_obj > MAX_MS_PER_OBJ) {
    results["DescriptorComputationTooSlow"] = true;
    return results;
  }
  (*eval_log_) << "Training set: " << endl;
  (*eval_log_) << train_->status("  ", true) << endl;

  // -- Update test set descriptors.
  updateDescriptors(config["Pipeline"], num_threads_, test_.get());
  (*eval_log_) << "Test set: " << endl;
  (*eval_log_) << test_->status("  ", true) << endl;
  
  // -- Train a classifier.
  GridClassifier::Ptr gc(new GridClassifier);
  string ncstr = config["GlobalParams"]["NumCells"].as<string>();
  istringstream iss(ncstr);
  vector<size_t> nc;
  while(!iss.eof()) {
    size_t buf;
    iss >> buf;
    nc.push_back(buf);
    cout << "NC: " << buf << endl;
  }
  gc->initialize(*train_, nc);
  GridClassifier::BoostingTrainer::Ptr trainer(new GridClassifier::BoostingTrainer(gc));
  trainer->obj_thresh_ = config["GlobalParams"]["ObjThresh"].as<double>();
  trainer->train(train_);
  gc->save(evalpath + "/classifier.gc");
  
  // -- Run evaluation.
  Evaluator ev(gc);
  ev.evaluateParallel(*test_);
  ev.plot_ = false;
  ev.saveResults(evalpath);

  results["Accuracy"] = ev.track_stats_.getTotalAccuracy();
  return results;
}

double JarvisTwiddler::objective(const YAML::Node& results) const
{
  if(results["NoDescriptors"].as<bool>() ||
     results["DescriptorComputationTooSlow"].as<bool>())
  {
    return std::numeric_limits<double>::infinity();    
  }

  double ms_per_obj = results["MsPerObj"].as<double>();
  double pct_error = (1.0 - results["Accuracy"].as<double>()) * 100.0;
  return pct_error + ms_per_obj;
}

void JarvisTwiddler::twiddleNumCells(YAML::Node config) const
{
  YAML::Node gp = config["GlobalParams"];

  vector<string> ncs;
  ncs.push_back("10");
  ncs.push_back("10 20");
  ncs.push_back("10 50");
  ncs.push_back("10 100");
  ncs.push_back("5 100");
    
  gp["NumCells"] = ncs[rand() % ncs.size()];

  ROS_ASSERT(config["GlobalParams"]["NumCells"].as<string>() == gp["NumCells"].as<string>());
}

void JarvisTwiddler::twiddleTrainerThreshold(YAML::Node config) const
{
  vector<double> thresholds;
  thresholds.push_back(50);
  thresholds.push_back(100);
  thresholds.push_back(200);

  config["GlobalParams"]["ObjThresh"] = thresholds[rand() % thresholds.size()];
}

void JarvisTwiddler::deleteRandomPod(YAML::Node config) const
{
  PipelineTwiddler::deleteRandomPod(config["Pipeline"], JarvisTwiddler::isRequired);
}

void JarvisTwiddler::addRawNormalizedHistogramBranch(YAML::Node config) const
{
  Pipeline pl(1);
  pl.deYAMLize(config["Pipeline"]);

  // -- If we don't have a BlobProjector, add one.
  if(!pl.hasPod<BlobProjector>()) {
    Pod* bp = pl.createPod("BlobProjector");
    pl.connect(bp->name() + ".Blob <- BlobEntryPoint.Blob");
  }

  Pod* bp = pl.pod<BlobProjector>();
  ROS_ASSERT(bp);
  Pod* ndh = pl.createPod("NormalizedDensityHistogram");
  vector<int> num_bins;
  num_bins.push_back(5);
  num_bins.push_back(10);
  num_bins.push_back(20);
  ndh->setParam<double>("NumBins", num_bins[rand() % num_bins.size()]);
  pl.connect(ndh->name() + ".Cloud <- " + bp->name() + ".Cloud");
  pl.connect("DescriptorAggregator.Descriptors <- " + ndh->name() + ".X");
  pl.connect("DescriptorAggregator.Descriptors <- " + ndh->name() + ".Y");
  pl.connect("DescriptorAggregator.Descriptors <- " + ndh->name() + ".Z");

  config["Pipeline"] = pl.YAMLize();
}

void JarvisTwiddler::addOrientedNormalizedHistogramBranch(YAML::Node config) const
{
  Pipeline pl(1);
  pl.deYAMLize(config["Pipeline"]);

  // -- If we don't have a BlobProjector, add one.
  if(!pl.hasPod<BlobProjector>()) {
    Pod* bp = pl.createPod("BlobProjector");
    pl.connect(bp->name() + ".Blob <- BlobEntryPoint.Blob");
  }
  Pod* bp = pl.pod<BlobProjector>();
  
  // -- If we don't have a CloudOrienter, add one.
  if(!pl.hasPod<CloudOrienter>()) {
    Pod* co = pl.createPod("CloudOrienter");
    pl.connect(co->name() + ".ProjectedBlob <- " + bp->name() + ".ProjectedBlob");
  }
  Pod* co = pl.pod<CloudOrienter>();

  ROS_ASSERT(bp && co);
  Pod* ndh = pl.createPod("NormalizedDensityHistogram");
  vector<int> num_bins;
  num_bins.push_back(5);
  num_bins.push_back(10);
  num_bins.push_back(20);
  ndh->setParam<double>("NumBins", num_bins[rand() % num_bins.size()]);
  pl.connect(ndh->name() + ".Cloud <- " + co->name() + ".OrientedCloud");
  pl.connect("DescriptorAggregator.Descriptors <- " + ndh->name() + ".X");
  pl.connect("DescriptorAggregator.Descriptors <- " + ndh->name() + ".Y");
  pl.connect("DescriptorAggregator.Descriptors <- " + ndh->name() + ".Z");

  config["Pipeline"] = pl.YAMLize();
}

