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
  double frac;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("input,i", bpo::value<string>(&input_path)->required())
    ("output,o", bpo::value<string>(&output_path)->default_value(""), "Overwrites original if output is not specified")
    ("frac,f", bpo::value<double>(&frac)->required())
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


  VectorXd weights = VectorXd::Ones(input.totalInstances());
  std::tr1::mt19937 mersenne;
  VectorXi indices(input.totalInstances() * frac);
  ee::weightedSampleLowVariance(weights, &mersenne, &indices);

  for(int i = 1; i < indices.rows(); ++i)
    ROS_ASSERT(indices(i) > indices(i-1));

  TrackDataset output;
  output.applyNameMappings(input);
  double idx = 0;
  for(size_t i = 0; i < input.size(); ++i) {
    const Dataset& track = input[i];
    for(size_t j = 0; j < track.size(); ++j, --idx) {
      if(idx < 1) {
        cout << "Saving track " << i << " frame " << j << endl;
        Dataset::Ptr t(new Dataset);
        t->applyNameMappings(output);
        t->instances_.push_back(track[j]);
        output.tracks_.push_back(t);
        idx += 1.0 / frac;
      }
    }
  }

  cout << "Input dataset: " << endl;
  cout << input.status("  ", false) << endl;
  cout << "Output dataset: " << endl;
  cout << output.status("  ", false) << endl;

  cout << "Saving to " << output_path << endl;
  output.save(output_path);
  
  return 0;
}
