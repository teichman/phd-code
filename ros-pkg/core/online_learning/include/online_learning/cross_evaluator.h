#include <online_learning/tbssl.h>
#include <boost/program_options.hpp>
#include <boost/foreach.hpp>

class CrossEvaluator
{
public:
  CrossEvaluator() {}
  void addTrackDataset(TrackDataset::ConstPtr td, std::string name);
  void evaluate(std::string dir, const std::vector<size_t>& num_cells) const;
  
protected:
  std::vector<TrackDataset::ConstPtr> tds_;
  std::vector<std::string> names_;
};
