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
  
  void Twiddler::run(std::string path, const Params& init)
  {
    ROS_ASSERT(!bfs::exists(path));
    bfs::create_directory(path);
    path_ = path;
    next_id_ = 0;
    
    if(!results_.count(init))
      results_[init] = evaluate(path_, init);

    twiddle();
  }

  void Twiddler::resume(std::string path)
  {
    ROS_ASSERT(bfs::exists(path));
    path_ = path;
    
    // -- Load everything from path.
    ROS_FATAL_STREAM("TODO");
    abort();
    // Set next_id_.
    
    twiddle();
  }

  void Twiddler::getBest(Params* best_params, Results* best_results) const
  {
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
      Results results = evaluate(path_, variation);
      results_[variation] = results;
      if(objective(results) < objective(best_results)) {
	best_params = variation;
	best_results = results;
	improvementHook(best_params, best_results);
      }

      // -- Save results.
      ostringstream oss;
      oss << path_ << "/" << setw(5) << setfill('0') << next_id_;
      string path = oss.str();
      ROS_ASSERT(!bfs::exists(path));
      bfs::create_directory(path);
      results.save(path + "/results.txt");
      variation.save(path + "/params.txt");
      ++next_id_;

      if(done(results))
	break;
    }
  }

  void Twiddler::improvementHook(const Params& params, const Results& results) const
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
