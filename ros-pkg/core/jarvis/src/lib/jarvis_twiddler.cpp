#include <online_learning/evaluator.h>
#include <jarvis/jarvis_twiddler.h>
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
  REGISTER_ACTION(JarvisTwiddler::addHogBranch);
  REGISTER_ACTION(JarvisTwiddler::replaceHogBranch);
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

  // Check memory usage.
  int retval = system(("free -m > " + evalpath + "/free.txt").c_str()); --retval;

  // -- Compute descriptors on training set.  If too slow, don't proceed.
  double ms_per_obj = updateDescriptors(config["Pipeline"], num_threads_, train_.get());
  results["MsPerObj"] = ms_per_obj;
  if(ms_per_obj > (double)MAX_MS_PER_OBJ) {
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
    if(!pl.hasPod<CloudOrienter>()) {
      Pod* co = pl.createPod("CloudOrienter");
      pl.connect(co->name() + ".ProjectedBlob <- BlobProjector.ProjectedBlob");
    }
    Pod* co = pl.pod<CloudOrienter>();
    
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

