#include <online_learning/tbssl.h>
#include <boost/program_options.hpp>
#include <boost/foreach.hpp>

using namespace std;
using namespace Eigen;
namespace bpo = boost::program_options;
namespace bfs = boost::filesystem;

// A k-window is a sequential subset of size k.
// Returns a label where each prediction is the maximum prediction across all k-windows.
// Intuitively, if a long track contains something that is non-X for a while, and then becomes X,
// this will return a label that is strongly X.  It will return a label that is strongly non-X
// only if the entire track is non-X.
Label classifyKWindows(const GridClassifier& gc, const Dataset& track, size_t k)
{
  if(track.size() < k)
    return gc.classifyTrack(track);
  
  vector<Label> predictions(track.size());
  for(size_t i = 0; i < track.size(); ++i)
    predictions[i] = gc.classify(track[i]);

  Label window = gc.prior();
  Label prediction = VectorXf::Ones(gc.prior().rows());
  prediction *= -numeric_limits<float>::max();
  for(size_t i = 0; i < track.size(); ++i) {
    window += predictions[i] / (float)k;
    if(i < k)
      continue;
    window -= predictions[i-k] / (float)k;
    for(int j = 0; j < window.rows(); ++j)
      prediction(j) = max(prediction(j), window(j));
  }

  return prediction;
}

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
  size_t max_tracks;
  string gc_path;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("tds", bpo::value(&td_paths)->required()->multitoken(), "List of TrackDatasets")
    ("pos", bpo::value(&pos)->multitoken())
    ("unk", bpo::value(&unk)->multitoken())
    ("neg", bpo::value(&neg)->multitoken())
    ("min-confidence", bpo::value(&min_confidence)->default_value(0), "")
    ("max-tracks", bpo::value(&max_tracks)->default_value(0), "0 -> no limit.")
    ("classifier", bpo::value(&gc_path)->default_value(""), "If not provided, use whatever classifications are in the TD already.")
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
    //cout << "  If you provide a classifier, it will label the tracks using the max label over all K-windows." << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  GridClassifier::Ptr gc;
  if(gc_path != "") {
    cout << "Loading classifier at " << gc_path << endl;
    gc = GridClassifier::Ptr(new GridClassifier);
    gc->load(gc_path);
  }
  
  TrackDataset filtered;
  for(size_t i = 0; i < td_paths.size(); ++i) {
    cout << "Working on " << td_paths[i] << endl;
    TrackDataset td;
    td.load(td_paths[i]);

    // Set the name mapping of filtered.
    if(i == 0) {
      if(gc)
        filtered.applyNameMapping("cmap", gc->nameMapping("cmap"));
      else
        filtered.applyNameMappings(td);
    }

    td.applyNameMappings(filtered);

    // -- Select the tracks that meet our criteria.
    size_t total_desired = 0;
    for(size_t j = 0; j < td.size(); ++j) {
      if(gc) {
        //td[j].setLabel(classifyKWindows(*gc, td[j], 30));
        td[j].setLabel(gc->classifyTrack(td[j]));
      }
      
      if(isDesired(td.label(j), td.nameMapping("cmap"), pos, unk, neg, min_confidence)) {
        ++total_desired;
        // Use a gross hack if there are more than the maximum number
        // we're willing to save.
        if(max_tracks == 0 || filtered.size() < max_tracks)
          filtered.tracks_.push_back(td.tracks_[j]);
        else if((double)rand() / RAND_MAX < (double)max_tracks / total_desired)
          filtered.tracks_[rand() % filtered.size()] = td.tracks_[j];
      }
    }
  }

  cout << "Filtered dataset: " << endl;
  cout << filtered.status("  ", true) << endl;
  filtered.save(output_path);
  cout << "Saved to " << output_path << endl;

  return 0;
}
