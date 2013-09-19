#include <online_learning/cross_validator.h>

using namespace std;
namespace bfs = boost::filesystem;
using namespace Eigen;

namespace odontomachus
{

  CrossValidator::CrossValidator(Dataset::ConstPtr data,
				 const std::set<std::string>& methods,
				 const std::string& output_path) :
    methods_(methods),
    output_path_(output_path),
    training_set_(new Dataset()),
    testing_set_(new Dataset())
  {
    // -- Initialize params.
    recognized_methods_.insert("stochastic_logistic");
    recognized_methods_.insert("vanilla_slr");
    int np[] = {0};
    num_projections_ = vector<int>(np, np + sizeof(np) / sizeof(int));
    //int nc[] = {10, 100, 1000};
    int nc[] = {200, 500, 1000};
    num_cells_ = vector<int>(nc, nc + sizeof(nc) / sizeof(int));
    int s[] = {1, 2, 3};
    smoothing_ = vector<int>(s, s + sizeof(s) / sizeof(int));
    
    // -- Check that classification methods are valid.
    set<string>::iterator it;
    for (it = methods.begin(); it != methods.end(); ++it) { 
      if (!recognized_methods_.count(*it))
	ROS_FATAL_STREAM("Requested classification method " << *it  << " is not recognized.  Aborting.");
    }

    // -- Set up output directory.
    if (!bfs::exists(output_path)) {
      ROS_INFO_STREAM("Creating output directory " << output_path);
      bfs::create_directory(output_path);
    }
    else if (!bfs::is_directory(output_path)) {
      ROS_FATAL_STREAM("Requested output dir " << output_path << " exists but is not a directory.  Aborting.");
    }
    else
      ROS_INFO_STREAM("Requested output dir " << output_path << " exists.  Adding new output to it.");

    // -- Set up the datasets.
    VectorXd prob(3);
    prob << 0.45, 0.45, 0.1;
    DatasetSplitter splitter(prob);
    if(getenv("SPLIT_FRAMEWISE"))
      splitter.splitFramewise(data);
    else
      splitter.splitTrackwise(data);
    training_set_ = splitter.partitions_[0];
    testing_set_ = splitter.partitions_[1];
    holdout_set_ = splitter.partitions_[2];

    ROS_ASSERT(training_set_ && testing_set_ && holdout_set_);
    ROS_DEBUG_STREAM("Training set: " << endl << training_set_->status() << flush);
    ROS_DEBUG_STREAM("Testing set: " << endl << testing_set_->status() << flush);
    ROS_DEBUG_STREAM("Holdout set: " << endl << holdout_set_->status() << flush);
  }

  void CrossValidator::run()
  {
    ROS_ASSERT(best_params_.empty());
    ROS_ASSERT(num_projections_.size() == 1 && num_projections_[0] == 0);
    if(methods_.count("stochastic_logistic")) {
      BOOST_FOREACH(int nc, num_cells_) {
	Params params;
	params["num_cells"] = nc;
	ROS_INFO_STREAM("Running test of stochastic logistic method with params: " << endl << params << flush);
	ProjectionSlicer::Ptr ps(new ProjectionSlicer());
	ps->initialize(nc, *training_set_);
	
	LogisticStochasticTrainer::Ptr trainer(new LogisticStochasticTrainer);
	ps->setTrainer(trainer);
	ps->train(*training_set_);
	ps->train(*holdout_set_);	      
	test("stochastic_logistic", ps, params);
      }
    }
    if(methods_.count("vanilla_slr")) {
      Params params;
      ROS_INFO_STREAM("Running test of vanilla stochastic logistic method with params: " << endl << params << flush);
      VanillaSLR::Ptr vslr(new VanillaSLR);
      vslr->applyNameMappings(*training_set_);
      vslr->train(*training_set_);
      vslr->train(*holdout_set_);
      test("vanilla_slr", vslr, params);
    }

    map<string, Params>::const_iterator it;
    for(it = best_params_.begin(); it != best_params_.end(); ++it) { 
      it->second.save(output_path_ + "/" + it->first + "/best_params.txt");
      map<string, PerfStats>::iterator sit = best_stats_.find(it->first);
      sit->second.save(output_path_ + "/" + it->first + "/best_results.txt");
    }
  }

  void CrossValidator::test(std::string method,
			    Classifier::Ptr classifier,
			    const Params& params)
  {
    // -- Set up output directory.
    bfs::create_directory(output_path_ + "/" + method);
    ostringstream oss;
    oss << output_path_ << "/" << method << "/";
    Params::const_iterator it;
    for(it = params.begin(); it != params.end(); ++it) { 
      oss << it->first << ":" << it->second; 
      if(it != --params.end())
	oss << "-";
    }
    string test_path = oss.str();
    bfs::create_directory(test_path);
    ROS_INFO_STREAM("Saving output to " << test_path << flush);

    // -- Run the test.
    PerfStats stats;
    stats.applyNameMapping("cmap", testing_set_->nameMapping("cmap"));
    for(int i = 0; i < testing_set_->labels_.rows(); ++i) {
      if(testing_set_->labels_(i) == -2)
	continue;
      
      Classification cl = classifier->classify(testing_set_->descriptors_.col(i));
      // cout << "----------" << endl;
      // cout << cl << endl;
      // cout << "Label: " << testing_set_->labels_(i) << endl;
      stats.incrementStats(testing_set_->labels_(i), cl.response_);
    }
    
    // -- Save results.
    map<string, PerfStats>::iterator sit = best_stats_.find(method);
    if(sit == best_stats_.end()) { 
      best_stats_.insert(pair<string, PerfStats>(method, stats));
      sit = best_stats_.find(method);
    }
    if(stats.getTotalAccuracy() >= sit->second.getTotalAccuracy()) {
      sit->second = stats;
      best_params_[method] = params;
      string classifier_path = test_path + "/classifier.ps";
      classifier->save(classifier_path);
      best_classifier_ = classifier;
      best_classifier_paths_[method] = classifier_path;
    }
    params.save(test_path + "/params.txt");
    training_set_->nameMapping("dmap").save(test_path + "/descriptor_map.txt");
    stats.save(test_path + "/results.txt");
    stats.saveConfusionMatrix(test_path + "/confusion.png");
    stats.saveConfusionMatrix(test_path + "/confusion.pdf");
    stats.savePrecisionRecallCurve(test_path + "/pr.png");
    stats.savePrecisionRecallCurve(test_path + "/pr.pdf");
  }

} // namespace
