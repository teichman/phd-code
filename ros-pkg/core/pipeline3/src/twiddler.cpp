#include <pipeline/twiddler.h>

using namespace std;
using boost::any;
namespace bfs = boost::filesystem;

namespace pipeline
{

  Twiddler::Twiddler() :
    next_id_(0)
  {
  }
  
  Params Twiddler::randomMerge(const Params& params0, const Params& params1)
  {
    Params params;
    map<string, any>::const_iterator it;
    for(it = params0.storage_.begin(); it != params0.storage_.end(); ++it) {
      string name = it->first;
      ROS_ASSERT(params1.exists(name));
      if(rand() % 2 == 0)
	params.storage_[name] = params0.storage_.find(name)->second;
      else
	params.storage_[name] = params1.storage_.find(name)->second;
    }
    return params;
  }

  double Twiddler::objective(const Results& results) const
  {
    return results["objective"];
  }
  
  void Twiddler::run(const Params& init, std::string rootpath)
  {
    ROS_ASSERT(!bfs::exists(rootpath));
    ROS_ASSERT(results_.empty());
    bfs::create_directory(rootpath);
    rootpath_ = rootpath;
    next_id_ = 0;

    // -- Run first evaluation on the initial set of params.
    ostringstream oss;
    oss << rootpath_ << "/" << setw(5) << setfill('0') << next_id_;
    string evalpath = oss.str();
    ROS_ASSERT(!bfs::exists(evalpath));
    bfs::create_directory(evalpath);
    results_[init] = evaluate(init, evalpath);
    improvementHook(init, results_[init], evalpath);
    results_[init].save(evalpath + "/results.txt");
    init.save(evalpath + "/params.txt");
    results_[init].save(rootpath_ + "/best_results.txt");
    init.save(rootpath_ + "/best_params.txt");
    ++next_id_;

    twiddle();
  }

  void Twiddler::resume(std::string rootpath)
  {
    ROS_ASSERT(bfs::exists(rootpath));
    rootpath_ = rootpath;
    
    // -- Load everything from rootpath.
    ROS_FATAL_STREAM("TODO");
    abort();
    // Set next_id_.
    
    twiddle();
  }

  void Twiddler::getBest(Params* best_params, Results* best_results) const
  {
    ROS_ASSERT(!results_.empty());
    double best_objective = numeric_limits<double>::max();
    map<Params, Results>::const_iterator it, best;
    for(it = results_.begin(); it != results_.end(); ++it) {
      if(objective(it->second) < best_objective) {
	best_objective = objective(it->second);
	best = it;
      }
    }
    *best_params = best->first;
    *best_results = best->second;
  }
  
  void Twiddler::twiddle()
  {
    Params best_params;
    Results best_results;
    getBest(&best_params, &best_results);
    
    while(true) {
      // -- Get another parameter variation.
      Params variation = generateParamVariation(best_params);
      if(results_.count(variation)) {
	continue;
      }

      // -- Evaluate and check for improvement.
      ostringstream oss;
      oss << rootpath_ << "/" << setw(5) << setfill('0') << next_id_;
      string evalpath = oss.str();
      ROS_ASSERT(!bfs::exists(evalpath));
      bfs::create_directory(evalpath);
      Results results = evaluate(variation, evalpath);
      results_[variation] = results;
      if(objective(results) < objective(best_results)) {
	best_params = variation;
	best_results = results;
	best_params.save(rootpath_ + "/best_params.txt");
	best_results.save(rootpath_ + "/best_results.txt");
	improvementHook(best_params, best_results, evalpath);
      }

      // -- Save results.
      results.save(evalpath + "/results.txt");
      variation.save(evalpath + "/params.txt");
      ++next_id_;

      if(done(results))
	break;
    }
  }

  void Twiddler::improvementHook(const Params& params, const Results& results, std::string evalpath) const
  {
    cout << "Twiddler found improvement!" << endl;
    cout << "New objective: " << objective(results) << endl;
    cout << "Results: " << endl;
    cout << results.status();
    cout << params << endl;
  }

  bool Twiddler::done(const Results& results) const
  {
    return false;
  }
  
}
