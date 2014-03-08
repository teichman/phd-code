#include <jarvis/inductor.h>
#include <online_learning/clusterer.h>
#include <boost/program_options.hpp>

using namespace std;
using namespace Eigen;

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  string gc_path;
  float intersection_threshold;
  int max_different_dspaces;
  string td_path;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("gc", bpo::value(&gc_path)->required(), "GridClassifier")
    ("intersection-threshold,i", bpo::value(&intersection_threshold)->required(), "")
    ("max-different-dspaces,m", bpo::value(&max_different_dspaces)->required(), "")
    ("td", bpo::value(&td_path)->required(), "TrackDataset.")
    ;

  p.add("gc", 1);
  p.add("intersection-threshold", 1);
  p.add("max-different-dspaces", 1);
  p.add("td", 1);

  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << argv[0] << " GC INTERSECTION_THRESHOLD MAX_DIFFERENT_DSPACES TD" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  cout << "Loading " << td_path << endl;
  TrackDataset td;
  td.load(td_path);

  cout << "Loading " << gc_path << endl;
  GridClassifier gc;
  gc.load(gc_path);

  VectorXi num = VectorXi::Zero(td.size());
  for(size_t i = 0; i < td.size(); ++i) {
    cout << i << " / " << td.size() << endl;
    for(size_t j = 0; j < td.size(); ++j)
      if(similar(td[i], td[j], gc, intersection_threshold, max_different_dspaces))
        ++num(i);
  }
  cout << num.transpose() << endl;

  int idx = -1;
  int maxnum = num.maxCoeff(&idx);
  cout << "Found a cluster of " << maxnum << " tracks." << endl;

  TrackDataset cluster;
  cluster.applyNameMappings(td);
  for(size_t i = 0; i < td.size(); ++i)
    if(similar(td[idx], td[i], gc, intersection_threshold, max_different_dspaces))
      cluster.tracks_.push_back(td.tracks_[i]);

  string filename = "cluster.td";
  cluster.save(filename);
  cout << "Saved to " << filename << endl;
  
  return 0;
}

