#include <matplotlib_interface/matplotlib_interface.h>  // Must come first because of Python.
#include <boost/program_options.hpp>
#include <online_learning/tbssl.h>

using namespace std;
using namespace Eigen;
namespace bpo = boost::program_options;
namespace bfs = boost::filesystem;

int main(int argc, char** argv)
{
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;
  
  // -- Parse args.
  string gc_path;
  string output_dir;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("gc", bpo::value<string>(&gc_path)->required(), "GridClassifier file")
    ("output", bpo::value<string>(&output_dir)->required(), "Directory to save output in")
    ;


  p.add("gc", 1);
  p.add("output", 1);

  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << argv[0] << " [OPTS] GRID_CLASSIFIER OUTPUT_DIR" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }
  
  GridClassifier gc;
  gc.load(gc_path);
  //cout << "GridClassifier: " << endl;
  //cout << gc.status("  ", true) << endl;

  
  mpliBegin();
  mpli("import matplotlib.pyplot as plt");
  mpli("import numpy as np");
  mpliPrintSize();

  // class/group/descriptor/elementXXX.png
  bfs::create_directory(output_dir);
  for(size_t c = 0; c < gc.nameMapping("cmap").size(); ++c) {
    string cname = gc.nameMapping("cmap").toName(c);
    bfs::create_directory(output_dir + "/" + cname);
    for(size_t g = 0; g < gc.grids_.size(); ++g) {
      ostringstream oss;
      oss << output_dir << "/" << cname << "/group" << setw(3) << setfill('0') << g;
      bfs::create_directory(oss.str());
      for(size_t d = 0; d < gc.grids_[g].size(); ++d) {
        ostringstream oss;
        oss << output_dir << "/" << cname << "/group" << setw(3) << setfill('0') << g << "/descriptor" << setw(3) << setfill('0') << d;
        bfs::create_directory(oss.str());
        for(size_t e = 0; e < gc.grids_[g][d].size(); ++e) {
          ostringstream oss;
          oss << output_dir << "/" << cname << "/group" << setw(3) << setfill('0') << g << "/descriptor" << setw(3) << setfill('0') << d << "/element" << setw(4) << setfill('0') << e;
          const Grid& grid = *gc.grids_[g][d][e];
          cout << oss.str() << endl;
          
          VectorXd weights = grid.cells_.row(c).cast<double>();
          VectorXd bins = grid.bins().cast<double>();
          double width = grid.width();
          double minval = bins.head(1)(0);
          double maxval = bins.tail(1)(0) + width;
          
          mpliExport(weights);
          mpliExport(bins);
          mpliExport(minval);
          mpliExport(maxval);
          mpliNamedExport("w", width);
          mpli("plt.bar(bins, weights, width=w)");
          mpli("plt.xlim(minval, maxval)");
          mpli("plt.ylim(-1, 1)");
          mpli("plt.ylabel('Weights')");
          mpli("plt.xlabel('Feature values')");
          mpliNamedExport("output_path", oss.str());
          mpli("plt.savefig(output_path + '.png')");
          mpli("plt.close()");
        }
      }
    }
  }
  
  return 0;
}

  
