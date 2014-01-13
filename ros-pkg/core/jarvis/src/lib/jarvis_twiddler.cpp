#include <online_learning/evaluator.h>
#include <jarvis/jarvis_twiddler.h>
#include <jarvis/descriptor_pipeline.h>

using namespace std;
using namespace pl;

/************************************************************
 * JarvisTwiddler
 ************************************************************/

JarvisTwiddler::JarvisTwiddler(vector<TrackDataset> datasets,
                               vector<Eigen::VectorXf> up_vectors,
                               int num_threads) :
  PipelineTwiddler(),
  num_threads_(num_threads),
  datasets_(datasets),
  up_vectors_(up_vectors)
{
  // -- Set up twiddle actions.
  REGISTER_ACTION(JarvisTwiddler::projectAllHOG);
  REGISTER_ACTION(JarvisTwiddler::projectAllHOGAndRemoveRaw);
  REGISTER_ACTION(JarvisTwiddler::projectRandomDescriptor);
  REGISTER_ACTION(JarvisTwiddler::deleteRandomProjector);
  REGISTER_ACTION(JarvisTwiddler::projectAllDescriptors);
  REGISTER_ACTION(JarvisTwiddler::projectAllDescriptorsConcatenated);
  REGISTER_ACTION(JarvisTwiddler::coupleAllDescriptorSpaces);
  
  // REGISTER_ACTION(JarvisTwiddler::deleteRandomPod);
  // REGISTER_ACTION(JarvisTwiddler::twiddleNumCells);
  // REGISTER_ACTION(JarvisTwiddler::twiddleTrainerThreshold);
  // REGISTER_ACTION(JarvisTwiddler::deleteRandomPod);
  // REGISTER_ACTION(JarvisTwiddler::addRawNormalizedHistogramBranch);
  // REGISTER_ACTION(JarvisTwiddler::addOrientedNormalizedHistogramBranch);
  // REGISTER_ACTION(JarvisTwiddler::addHogBranch);
  // REGISTER_ACTION(JarvisTwiddler::replaceHogBranch);
  
  // registerAction("twiddleRandomHistogramNumBins",
  //                boost::bind(&PipelineTwiddler::twiddlePodParamsLockstep<NormalizedDensityHistogram, double>, *this, _1,
  //                            "NumBins", vector<double>{5, 10, 20}));
  // registerAction("twiddleRandomDynamicImageWindowHeightPercent",
  //                boost::bind(&PipelineTwiddler::twiddlePodParam<DynamicImageWindow, double>, *this, _1,
  //                            "HeightPercent", vector<double>{0.2, 0.5, 1.0}));
  // registerAction("twiddleRandomDynamicImageWindowWidthPercent",
  //                boost::bind(&PipelineTwiddler::twiddlePodParam<DynamicImageWindow, double>, *this, _1,
  //                            "WidthPercent", vector<double>{0.2, 0.5, 1.0}));
  // registerAction("twiddleRandomDynamicImageWindowVerticalAlignment",
  //                boost::bind(&PipelineTwiddler::twiddlePodParam<DynamicImageWindow, double>, *this, _1,
  //                            "VerticalAlignment", vector<string>{"Top", "Center", "Bottom"}));
  // registerAction("twiddleRandomDynamicImageWindowHorizontalAlignment",
  //                boost::bind(&PipelineTwiddler::twiddlePodParam<DynamicImageWindow, double>, *this, _1,
  //                            "HorizontalAlignment", vector<string>{"Left", "Center", "Right"}));
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
  ROS_ASSERT(config["Pipeline"]);
  Twiddler::improvementHook(config, results, evalpath);
  Pipeline pl(1);
  pl.deYAMLize(config["Pipeline"]);
  pl.writeGraphviz(root_dir_ + "/best_pipeline.gv");
}

double JarvisTwiddler::runSingleEval(const YAML::Node& config,
                                     const GridClassifier& base_classifier,
                                     TrackDataset::ConstPtr train,
                                     TrackDataset::ConstPtr test) const
{
  // -- Train a classifier.
  GridClassifier::Ptr gc(new GridClassifier(base_classifier));
  GridClassifier::BoostingTrainer::Ptr trainer(new GridClassifier::BoostingTrainer(gc));
  trainer->obj_thresh_ = config["GlobalParams"]["ObjThresh"].as<double>();
  vector<TrackDataset::ConstPtr> datasets; datasets.push_back(train);
  vector<Indices> indices; indices.push_back(Indices::All(train->size()));
  trainer->train(datasets, indices);
  
  // -- Run evaluation.
  Evaluator ev(gc);
  ev.evaluateParallel(*test);

  return ev.track_stats_.getTotalAccuracy();
}

void JarvisTwiddler::splitDataset(const TrackDataset& td, double pct0, TrackDataset* split0, TrackDataset* split1) const
{
  ROS_ASSERT(!td.empty());
  ROS_ASSERT(pct0 >= 0 && pct0 <= 1);

  // -- Copy name mappings.
  split0->applyNameMappings(td);
  split1->applyNameMappings(td);

  // -- Get a random ordering.
  vector<size_t> index(td.size());
  for(size_t i = 0; i < td.size(); ++i)
    index[i] = i;
  random_shuffle(index.begin(), index.end());

  // -- Split.
  split0->tracks_.reserve(td.size());
  split1->tracks_.reserve(td.size());
  for(size_t i = 0; i < index.size(); ++i) {
    if((double)i / index.size() < pct0)
      split0->tracks_.push_back(td.tracks_[index[i]]);
    else
      split1->tracks_.push_back(td.tracks_[index[i]]);
  }
}

double JarvisTwiddler::runMultipleEvals(std::string debugging_name,
                                        const YAML::Node& config,
                                        const GridClassifier& base_classifier,
                                        const TrackDataset& dataset,
                                        int num_evals, double pct_train) const
{
  double accuracy = 0;
  for(int i = 0; i < num_evals; ++i) {
    (*eval_log_) << "== " << debugging_name << " " << i << " == " << endl;
    TrackDataset::Ptr train(new TrackDataset);
    TrackDataset::Ptr test(new TrackDataset);
    splitDataset(dataset, pct_train, train.get(), test.get());

    (*eval_log_) << "Train: " << endl;
    (*eval_log_) << train->status("  ", true) << endl;
    (*eval_log_) << "Test: " << endl;
    (*eval_log_) << test->status("  ", true) << endl;

    double acc = runSingleEval(config, base_classifier, train, test);
    (*eval_log_) << debugging_name << " " << i << " eval accuracy: " << acc << endl;
    accuracy += acc;
  }
  return accuracy / num_evals;
}

void JarvisTwiddler::initializeBaseClassifiers(const YAML::Node& config)
{
  base_classifiers_.clear();
  for(size_t i = 0; i < datasets_.size(); ++i) {
    GridClassifier gc;
    string ncstr = config["GlobalParams"]["NumCells"].as<string>();
    istringstream iss(ncstr);
    vector<size_t> nc;
    while(!iss.eof()) {
      size_t buf;
      iss >> buf;
      nc.push_back(buf);
    }
    gc.initialize(datasets_[i], nc);
    base_classifiers_.push_back(gc);
  }
}

YAML::Node JarvisTwiddler::evaluate(const YAML::Node& config, std::string evalpath)
{
  YAML::Node results;
  results["NoDescriptors"] = false;
  results["DescriptorComputationTooSlow"] = false;
  
  // -- Save the pipeline we are evaluating in graphviz format.
  Pipeline pl(1);
  ROS_ASSERT(config["Pipeline"]);
  pl.deYAMLize(config["Pipeline"]);
  pl.writeGraphviz(evalpath + "/pipeline.gv");
  if(pl.pod<DescriptorAggregator>()->dmap().size() == 0) {
    results["NoDescriptors"] = true;
    return results;
  }

  // -- Check memory usage.
  int retval = system(("free -m > " + evalpath + "/free.txt").c_str()); --retval;
  
  // -- Compute new descriptors for each class problem.  If too slow, don't proceed.
  double ms_per_obj = 0;
  for(size_t i = 0; i < datasets_.size(); ++i) {
    ms_per_obj += updateDescriptors(config["Pipeline"], num_threads_, &datasets_[i], up_vectors_[i]) / datasets_.size();
    (*eval_log_) << "==================== Test " << i << " " << endl;
    (*eval_log_) << "Dataset: " << endl;
    (*eval_log_) << datasets_[i].status("  ", true) << endl;
  }
  results["MsPerObj"] = ms_per_obj;
  if(ms_per_obj > (double)MAX_MS_PER_OBJ) {
    results["DescriptorComputationTooSlow"] = true;
    return results;
  }

  // -- Initialize the base classifiers.
  initializeBaseClassifiers(config);
  
  // -- Run accuracy evaluations for each test.
  double large_eval_accuracy = 0;
  double tiny_eval_accuracy = 0;
  for(size_t i = 0; i < datasets_.size(); ++i) {
    large_eval_accuracy += runMultipleEvals("LargeEval", config,
                                            base_classifiers_[i], datasets_[i],
                                            20, 0.5);
    tiny_eval_accuracy += runMultipleEvals("TinyEval", config, base_classifiers_[i],
                                           datasets_[i], 200, 30.0 / datasets_[i].size());
  }
  results["LargeEvalAccuracy"] = large_eval_accuracy / datasets_.size();
  results["TinyEvalAccuracy"] = tiny_eval_accuracy / datasets_.size();
  
  double large_eval_error = 1.0 - results["LargeEvalAccuracy"].as<double>();
  double tiny_eval_error = 1.0 - results["TinyEvalAccuracy"].as<double>();
  results["MeanError"] = (large_eval_error + tiny_eval_error) / 2;
  
  return results;
}

double JarvisTwiddler::objective(const YAML::Node& results) const
{
  if(results["NoDescriptors"].as<bool>() ||
     results["DescriptorComputationTooSlow"].as<bool>())
  {
    return std::numeric_limits<double>::infinity();    
  }
  return results["MeanError"].as<double>();
}

void JarvisTwiddler::twiddleNumCells(YAML::Node config) const
{
  ROS_ASSERT(config["GlobalParams"]);
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
  ROS_ASSERT(config["Pipeline"]);
  PipelineTwiddler::deleteRandomPod(config, JarvisTwiddler::isRequired);
}

void JarvisTwiddler::replaceHogBranch(YAML::Node config) const
{
  ROS_ASSERT(config["Pipeline"]);
  Pipeline pl(1);
  pl.deYAMLize(config["Pipeline"]);
  ROS_ASSERT(!pl.pods().empty());

  // -- Delete random Hog.
  vector<HogArray*> has = pl.filterPods<HogArray>();
  if(has.empty())
    return;

  HogArray* ha = has[rand() % has.size()];
  pl.deletePod(ha->name());
  pl.prune(JarvisTwiddler::isRequired);
  config["Pipeline"] = pl.YAMLize();

  // -- Add new Hog.
  addHogBranch(config);
}

void JarvisTwiddler::addHogBranch(YAML::Node config) const
{
  ROS_ASSERT(config["Pipeline"]);
  Pipeline pl(1);
  pl.deYAMLize(config["Pipeline"]);

  // -- Choose a CloudProjector plane.
  vector<string> views {"XY", "XZ", "YZ"};
  string view = views[rand() % views.size()];
  Pod* cp = NULL;
  vector<CloudProjector*> cps = pl.filterPods<CloudProjector>();
  for(size_t i = 0; i < cps.size(); ++i)
    if(cps[i]->param<string>("View") == view)
      cp = cps[i];

  // -- If we don't have the chosen one, add it.
  if(!cp) {
    cp = pl.createPod("CloudProjector");
    cp->setParam<string>("View", view);
    vector<double> ppms {10, 20, 40};
    cp->setParam("PixelsPerMeter", ppms[rand() % ppms.size()]);
    vector<double> nrs {10, 20, 40};
    cp->setParam("NumRows", nrs[rand() % nrs.size()]);
    cp->setParam("NumCols", nrs[rand() % nrs.size()]);
    cp->setParam<double>("MinIntensity", 1);
    cp->setParam<double>("KernelSize", 1);

    // -- If we don't have a CloudOrienter, add one.
    if(!pl.hasPod<GravitationalCloudOrienter>()) {
      Pod* co = pl.createPod("GravitationalCloudOrienter");
      pl.connect(co->name() + ".ProjectedBlob <- BlobProjector.ProjectedBlob");
    }
    Pod* co = pl.pod<GravitationalCloudOrienter>();
    
    pl.connect(cp->name() + ".Cloud <- " + co->name() + ".OrientedCloud");
  }

  // -- Add a DynamicImageWindow for this HogArray.
  vector<string> vas;
  vas.push_back("Top");
  vas.push_back("Center");
  vas.push_back("Bottom");
  vector<string> has;
  has.push_back("Left");
  has.push_back("Center");
  has.push_back("Right");
  vector<double> pcts;
  pcts.push_back(0.2);
  pcts.push_back(0.5);
  pcts.push_back(1.0);
    
  Pod* dw = pl.createPod("DynamicImageWindow");
  dw->setParam<double>("Scaling", 1);
  dw->setParam<double>("HeightPercent", pcts[rand() % pcts.size()]);
  dw->setParam<double>("WidthPercent", pcts[rand() % pcts.size()]);
  dw->setParam<string>("HorizontalAlignment", has[rand() % has.size()]);
  dw->setParam<string>("VerticalAlignment", vas[rand() % vas.size()]);
  pl.connect(dw->name() + ".Image <- " + cp->name() + ".Image");

  // -- Add a HOGArray.
  vector<double> nbs;
  nbs.push_back(8);
  nbs.push_back(6);
  nbs.push_back(4);
  vector<double> wszs;
  wszs.push_back(20);
  wszs.push_back(10);
  double sz = wszs[rand() % wszs.size()];
  if(sz > dw->param<double>("HeightPercent") * cp->param<double>("NumRows"))
    return;
  if(sz > dw->param<double>("WidthPercent") * cp->param<double>("NumCols"))
    return;
  
  vector<double> cszs;
  cszs.push_back(5);
  cszs.push_back(10);
  
  Pod* hog = pl.createPod("HogArray");
  hog->setParam<string>("UVPattern", "Center");
  hog->setParam<double>("WindowWidth", sz);
  hog->setParam<double>("WindowHeight", sz);
  hog->setParam<double>("BlockWidth", sz);
  hog->setParam<double>("BlockHeight", sz);
  hog->setParam<double>("BlockStride", 10);
  hog->setParam<double>("CellSize", cszs[rand() % cszs.size()]);
  hog->setParam<double>("NumBins", nbs[rand() % nbs.size()]);
  pl.connect(hog->name() + ".Image <- " + dw->name() + ".Image");
  pl.connect("DescriptorAggregator.Descriptors <- " + hog->name() + ".ConcatenatedDescriptors");
  
  config["Pipeline"] = pl.YAMLize();
}

void JarvisTwiddler::projectRandomDescriptor(YAML::Node config) const
{
  ROS_ASSERT(config["Pipeline"]);
  Pipeline pl(1);
  pl.deYAMLize(config["Pipeline"]);
  
  // -- Get a random descriptor going into DescriptorAggregator.
  Pod* da = pl.pod<DescriptorAggregator>();
  ROS_ASSERT(da);
  vector<const Outlet*> outlets = da->inputPipes("Descriptors");
  vector<const Outlet*> valid; valid.reserve(outlets.size());
  for(size_t i = 0; i < outlets.size(); ++i) {
    if(outlets[i]->pod()->isPodType<HogArray>())
      continue;
    if(outlets[i]->pod()->isPodType<RandomProjector>())
      continue;
    valid.push_back(outlets[i]);
  }
  if(valid.empty())
    return;
  const Outlet* outlet = valid[rand() % valid.size()];
  Pod* pod = outlet->pod();

  // -- Remove any existing RPs attached to that Descriptor.
  vector<RandomProjector*> rps = pl.filterPods<RandomProjector>();
  for(size_t i = 0; i < rps.size(); ++i)
    if(rps[i]->hasParent(pod->name()))
      pl.deletePod(rps[i]->name());
  rps.clear();
  
  // -- Add a RandomProjector.
  vector<int> nps {5, 10, 20};
  int np = nps[rand() % nps.size()];
  Pod* rp = pl.createPod("RandomProjector");
  // We use the unique Pod hash as the seed so that we don't use the same seed everywhere
  // and that we don't keep trying configurations that are identical other than the
  // change in random seed.  The % 10000 is so that we don't get very large seeds which
  // get serialize / deserialize properly.
  rp->setParam<double>("Seed", pod->getUniqueHash() % 10000);
  rp->setParam<double>("NumProjections", np);
  pl.connect(rp->name() + ".Descriptor <- " + outlet->address());
  pl.connect("DescriptorAggregator.Descriptors <- " + rp->name() + ".Projected");
  
  config["Pipeline"] = pl.YAMLize();
}

void JarvisTwiddler::deleteRandomProjector(YAML::Node config) const
{
  ROS_ASSERT(config["Pipeline"]);
  Pipeline pl(1);
  pl.deYAMLize(config["Pipeline"]);

  vector<RandomProjector*> rps = pl.filterPods<RandomProjector>();
  if(rps.empty())
    return;
  RandomProjector* rp = rps[rand() % rps.size()];
  pl.deletePod(rp->name());
  
  config["Pipeline"] = pl.YAMLize();
}

void JarvisTwiddler::projectAllDescriptors(YAML::Node config) const
{
  ROS_ASSERT(config["Pipeline"]);
  Pipeline pl(1);
  pl.deYAMLize(config["Pipeline"]);

  // -- Get rid of any existing RandomProjectors.
  vector<RandomProjector*> rps = pl.filterPods<RandomProjector>();
  for(size_t i = 0; i < rps.size(); ++i)
    pl.deletePod(rps[i]->name());
  
  // -- Add new RandomProjectors for all descriptors.
  vector<int> nps {2, 5, 10, 20};
  int np = nps[rand() % nps.size()];

  Pod* da = pl.pod<DescriptorAggregator>();
  ROS_ASSERT(da);
  vector<const Outlet*> outlets = da->inputPipes("Descriptors");
  for(size_t i = 0; i < outlets.size(); ++i) {
    Pod* rp = pl.createPod("RandomProjector");
    rp->setParam<double>("Seed", outlets[i]->pod()->getUniqueHash() % 10000);
    rp->setParam<double>("NumProjections", np);
    pl.connect(rp->name() + ".Descriptor <- " + outlets[i]->address());
    pl.connect("DescriptorAggregator.Descriptors <- " + rp->name() + ".Projected");
  }
  
  config["Pipeline"] = pl.YAMLize();
}

void JarvisTwiddler::projectAllDescriptorsConcatenated(YAML::Node config) const
{
  projectAllDescriptors(config);
  
  ROS_ASSERT(config["Pipeline"]);
  Pipeline pl(1);
  pl.deYAMLize(config["Pipeline"]);
  
  // -- Disconnect all existing RandomProjectors.
  vector<RandomProjector*> rps = pl.filterPods<RandomProjector>();
  for(size_t i = 0; i < rps.size(); ++i)
    pl.disconnect("DescriptorAggregator.Descriptors <- " + rps[i]->name() + ".Projected");

  // -- Concatenate all RandomProjectors.
  Pod* dc = pl.createPod("DescriptorConcatenator");
  for(size_t i = 0; i < rps.size(); ++i)
    pl.connect(dc->name() + ".Descriptors <- " + rps[i]->name() + ".Projected");
  pl.connect("DescriptorAggregator.Descriptors <- " + dc->name() + ".ConcatenatedDescriptors");

  config["Pipeline"] = pl.YAMLize();
}

void JarvisTwiddler::coupleAllDescriptorSpaces(YAML::Node config) const
{
  projectAllDescriptorsConcatenated(config);

  ROS_ASSERT(config["Pipeline"]);
  Pipeline pl(1);
  pl.deYAMLize(config["Pipeline"]);

  vector<DescriptorConcatenator*> dcs = pl.filterPods<DescriptorConcatenator>();
  ROS_ASSERT(!dcs.empty());
  DescriptorConcatenator* dc = dcs[rand() % dcs.size()];

  pl.disconnect("DescriptorAggregator.Descriptors <- " + dc->name() + ".ConcatenatedDescriptors");
  
  vector<int> nps {5, 10, 20, 50};
  int np = nps[rand() % nps.size()];

  Pod* rp = pl.createPod("RandomProjector");
  rp->setParam<double>("Seed", dc->getUniqueHash() % 10000);
  rp->setParam<double>("NumProjections", np);
  pl.connect(rp->name() + ".Descriptor <- " + dc->name() + ".ConcatenatedDescriptors");
  pl.connect("DescriptorAggregator.Descriptors <- " + rp->name() + ".Projected");

  config["Pipeline"] = pl.YAMLize();
}

void JarvisTwiddler::projectAllHOGAndRemoveRaw(YAML::Node config) const
{
  projectAllHOG(config);

  ROS_ASSERT(config["Pipeline"]);
  Pipeline pl(1);
  pl.deYAMLize(config["Pipeline"]);

  // -- Remove raw HOG descriptors.
  Pod* da = pl.pod<DescriptorAggregator>();
  ROS_ASSERT(da);
  vector<const Outlet*> outlets = da->inputPipes("Descriptors");
  for(size_t i = 0; i < outlets.size(); ++i)
    if(outlets[i]->pod()->isPodType<HogArray>())
      pl.disconnect("DescriptorAggregator.Descriptors <- " + outlets[i]->address());
  
  config["Pipeline"] = pl.YAMLize();
}

void JarvisTwiddler::projectAllHOG(YAML::Node config) const
{
  ROS_ASSERT(config["Pipeline"]);
  Pipeline pl(1);
  pl.deYAMLize(config["Pipeline"]);

  // -- Get rid of any RandomProjectors already attached to HogArrays.
  vector<RandomProjector*> rps = pl.filterPods<RandomProjector>();
  for(size_t i = 0; i < rps.size(); ++i)
    if(rps[i]->hasParent<HogArray>())
      pl.deletePod(rps[i]->name());
  
  // -- Add new RandomProjectors.
  vector<int> nps {5, 10, 20, 50};
  int np = nps[rand() % nps.size()];
  vector<HogArray*> hogs = pl.filterPods<HogArray>();
  for(size_t i = 0; i < hogs.size(); ++i) {
    Pod* rp = pl.createPod("RandomProjector");
    rp->setParam<double>("Seed", i);
    rp->setParam<double>("NumProjections", np);
    pl.connect(rp->name() + ".Descriptor <- " + hogs[i]->name() + ".ConcatenatedDescriptors");
    pl.connect("DescriptorAggregator.Descriptors <- " + rp->name() + ".Projected");
  }
  
  config["Pipeline"] = pl.YAMLize();
}

void JarvisTwiddler::addRawNormalizedHistogramBranch(YAML::Node config) const
{
  Pipeline pl(1);
  ROS_ASSERT(config["Pipeline"]);
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
  ROS_ASSERT(config["Pipeline"]);
  Pipeline pl(1);
  pl.deYAMLize(config["Pipeline"]);

  // -- If we don't have a BlobProjector, add one.
  if(!pl.hasPod<BlobProjector>()) {
    Pod* bp = pl.createPod("BlobProjector");
    pl.connect(bp->name() + ".Blob <- BlobEntryPoint.Blob");
  }
  Pod* bp = pl.pod<BlobProjector>();
  
  // -- If we don't have a GravitationalCloudOrienter, add one.
  if(!pl.hasPod<GravitationalCloudOrienter>()) {
    Pod* co = pl.createPod("GravitationalCloudOrienter");
    pl.connect(co->name() + ".ProjectedBlob <- " + bp->name() + ".ProjectedBlob");
  }
  Pod* co = pl.pod<GravitationalCloudOrienter>();

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


void evaluateConfig(YAML::Node config, int num_threads,
                    TrackDataset::Ptr train, TrackDataset::Ptr test,
                    GridClassifier::Ptr* gcp,
                    Evaluator::Ptr* evp)
{
  // -- Update descriptors on the datasets.
  updateDescriptors(config["Pipeline"], num_threads, train.get());
  updateDescriptors(config["Pipeline"], num_threads, test.get());

  // -- Initialize the classifier
  srand(time(NULL));
  GridClassifier::Ptr gc(new GridClassifier);
  string ncstr = config["GlobalParams"]["NumCells"].as<string>();
  istringstream iss(ncstr);
  vector<size_t> nc;
  while(!iss.eof()) {
    size_t buf;
    iss >> buf;
    nc.push_back(buf);
  }
  gc->initialize(*train, nc);

  // -- Train.
  GridClassifier::BoostingTrainer::Ptr trainer(new GridClassifier::BoostingTrainer(gc));
  trainer->obj_thresh_ = config["GlobalParams"]["ObjThresh"].as<double>();
  trainer->train(train);
  
  // -- Evaluate.
  Evaluator::Ptr ev(new Evaluator(gc));
  ev->evaluateParallel(*test);

  *gcp = gc;
  *evp = ev;
}
