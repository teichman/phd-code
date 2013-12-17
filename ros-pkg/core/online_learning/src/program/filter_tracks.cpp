#include <online_learning/tbssl.h>
#include <boost/program_options.hpp>
#include <boost/foreach.hpp>

using namespace std;
using namespace Eigen;
namespace bpo = boost::program_options;
namespace bfs = boost::filesystem;

bool isDesired(const Label& label, const NameMapping& cmap,
               const vector<string>& pos,
               const vector<string>& unk,
               const vector<string>& neg,
               float min_confidence)
{
  for(size_t i = 0; i < pos.size(); ++i) {
    ROS_ASSERT(cmap.hasName(pos[i]));
    if(label(cmap.toId(pos[i])) <= min_confidence)
      return false;
  }
  for(size_t i = 0; i < unk.size(); ++i) {
    ROS_ASSERT(cmap.hasName(unk[i]));
    if(label(cmap.toId(unk[i])) != 0)
      return false;
  }
  for(size_t i = 0; i < neg.size(); ++i) {
    ROS_ASSERT(cmap.hasName(neg[i]));
    if(label(cmap.toId(neg[i])) >= -min_confidence)
      return false;
  }
  return true;
}

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  vector<string> td_paths;
  vector<string> pos;
  vector<string> unk;
  vector<string> neg;
  string output_path;
  float min_confidence;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("tds", bpo::value(&td_paths)->required()->multitoken(), "List of TrackDatasets")
    ("pos", bpo::value(&pos)->multitoken())
    ("unk", bpo::value(&unk)->multitoken())
    ("neg", bpo::value(&neg)->multitoken())
    ("min-confidence", bpo::value(&min_confidence)->default_value(0), "")
    ("output,o", bpo::value(&output_path), "")
    ;

  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << argv[0] << " [OPTS] --pos CLASS [ CLASS ... ] --unk CLASS [ CLASS ... ] --neg CLASS [ CLASS ... ] --tds TD [ PATH ... ] -o OUTPUT" << endl;
    cout << "  Saves a new TD to OUTPUT that consists of all tracks in that meet the conjunction of all criteria." << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  TrackDataset filtered;
  
  for(size_t i = 0; i < td_paths.size(); ++i) {
    cout << "Working on " << td_paths[i] << endl;
    TrackDataset td;
    td.load(td_paths[i]);
    if(i == 0)
      filtered.applyNameMappings(td);
    else
      td.applyNameMappings(filtered);

    // -- Select the tracks that meet our criteria.
    for(size_t j = 0; j < td.size(); ++j)
      if(isDesired(td.label(j), td.nameMapping("cmap"), pos, unk, neg, min_confidence))
        filtered.tracks_.push_back(td.tracks_[j]);
  }

  cout << "Filtered dataset: " << endl;
  cout << filtered.status("  ", true) << endl;
  filtered.save(output_path);
  cout << "Saved to " << output_path << endl;

  return 0;
}
