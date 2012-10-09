#include <pipeline/twiddler.h>

using namespace std;
using boost::any;

namespace pipeline
{

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

  bool Twiddler::isImprovement(const Params& prev, const Params& curr) const
  {
    return (curr.get<double>("objective") < prev.get<double>("objective"));
  }

  void Twiddler::run(const Params& init, std::string output_path)
  {
    ROS_ASSERT(!bfs::exists(output_path));
    bfs::create_directory(output_path);

    Params best_params = init;
    Params best_results = evaluate(init);
    while(true) {
      Params variation = generateParamVariation(best_params);
      if(results_.count(variation))
	continue;
      results_[variation] = evaluate(variation);
      if(isImprovement(best_results, results_[variation])) {
	best_params = variation;
	best_results = results_[variation];
      }

      // -- Save these results to output_path.
    }
  }
  
}
