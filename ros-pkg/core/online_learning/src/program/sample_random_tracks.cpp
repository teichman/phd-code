#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <eigen_extensions/random.h>
#include <online_learning/grid_classifier.h>

using namespace std;
using namespace Eigen;
namespace ee = eigen_extensions;

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  namespace bfs = boost::filesystem;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  string input_path;
  string output_path;
  size_t max_track_length;
  double frac;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("input,i", bpo::value<string>(&input_path)->required())
    ("output,o", bpo::value<string>(&output_path)->default_value(""), "Overwrites original if output is not specified")
    ("max-track-length", bpo::value<size_t>(&max_track_length)->default_value(0), "If set, split the dataset into smaller tracks with length at most this.")
    ("frac,f", bpo::value<double>(&frac)->required(), "If splitting, the fraction is applied to the post-split dataset.")
    ;

  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << argv[0] << " [OPTS] -i TD -f FRAC [ -o OUTPUT ]" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  ROS_ASSERT(frac > 0 && frac < 1);

  cout << "Loading " << input_path << "." << endl;
  TrackDataset input;
  input.load(input_path);
  cout << "Input dataset: " << endl;
  cout << input.status("  ", false) << endl;

  if(max_track_length > 0) {
    cout << "Splitting dataset." << endl;
    splitTracks(max_track_length, &input);
    cout << "After splitting: " << endl;
    cout << input.status("  ", false) << endl;
  }
  
  VectorXd weights = VectorXd::Ones(input.size());
  std::tr1::mt19937 mersenne;
  VectorXi indices(input.size() * frac);
  ee::weightedSampleLowVariance(weights, &mersenne, &indices);

  TrackDataset output;
  output.applyNameMappings(input);
  for(int i = 0; i < indices.rows(); ++i)
    output.tracks_.push_back(input.tracks_[indices[i]]);

  cout << "Output dataset: " << endl;
  cout << output.status("  ", false) << endl;

  cout << "Saving to " << output_path << endl;
  output.save(output_path);
  
  return 0;
}
