#include <pipeline/twiddler.h>

using namespace std;
using boost::any;

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
	params.storage_[name] = params0.storage_[name];
      else
	params.storage_[name] = params1.storage_[name];
    }
    return params;
  }

  void Twiddler::initialize(std::string path)
  {
    ROS_ASSERT(!bfs::exists(path));
    bfs::create_directory(path);
    next_id_ = 0;
    path_ = path;
  }

  void Twiddler::load(std::string path)
  {
    
  }

  double Twiddler::objective(const Results& results) const
  {
    return results["objective"];
  }
  
  void Twiddler::run(const Params& init)
  {
    Params best_params = init;
    Params best_results = evaluate(init);
    while(true) {
      // -- Get another parameter variation.
      Params variation = generateParamVariation(best_params);
      if(results_.count(variation))
	continue;

      // -- Evaluate and check for improvement.
      Results results = evaluate(variation);
      results_[variation] = results;
      if(objective(results) < objective(best_results)) {
	best_params = variation;
	best_results = results;
      }

      // -- Save results to output_path.
      ostringstream oss;
      oss << path << "/" << setw(5) << setfill('0') << next_id_;
      string path = oss.str();
      ROS_ASSERT(!bfs::exists(path));
      bfs::create_directory(path);
      results.save(path + "/results.txt");
      variation.save(path + "/params.txt");
      ++next_id_;
    }
  }
  
}
