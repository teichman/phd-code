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
  size_t gid, did, eid, cid;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("gc", bpo::value<string>()->required(), "GridClassifier file")
    ("gid", bpo::value<size_t>(&gid)->required(), "Which group of grids.")
    ("did", bpo::value<size_t>(&did)->required(), "Which descriptor.")
    ("eid", bpo::value<size_t>(&eid)->required(), "Which element of that descriptor.")
    ("cid", bpo::value<size_t>(&cid)->required(), "Which class.")
    ;


  p.add("gc", 1);
  p.add("gid", 1);
  p.add("did", 1);
  p.add("eid", 1);
  p.add("cid", 1);

  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << argv[0] << " [OPTS] GRID_CLASSIFIER GROUP DESCRIPTOR ELEMENT CLASS" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }
  
  GridClassifier gc;
  gc.load(opts["gc"].as<string>());
  //cout << "GridClassifier: " << endl;
  //cout << gc.status("  ", true) << endl;

  ostringstream oss;
  oss << "group" << gid << "descr" << setw(2) << setfill('0') << did << "ele" << setw(4) << setfill('0') << eid << "-" << gc.nameMapping("cmap").toName(cid);
  string output_path = oss.str();
  cout << "Saving output to basename " << output_path << endl;

  cout << "Showing element " << eid << " of descriptor space " << gc.nameMapping("dmap").toName(did) << endl;
  cout << "Class: " << gc.nameMapping("cmap").toName(cid) << endl;

  ROS_ASSERT(gid < gc.grids_.size());
  ROS_ASSERT(did < gc.grids_[gid].size());
  ROS_ASSERT(eid < gc.grids_[gid][did].size());
  const Grid& grid = *gc.grids_[gid][did][eid];
  VectorXd weights = grid.cells_.row(cid).cast<double>();
  VectorXd bins = grid.bins().cast<double>();
  double width = grid.width();
  double minval = bins.head(1)(0);
  double maxval = bins.tail(1)(0) + width;

  cout << "Bin locations: " << endl << bins.transpose() << endl;
  cout << "Bin weights: " << endl << weights.transpose() << endl;
  cout << "Bin width: " << width << endl;
  cout << "Minimum: " << bins.head(1) << endl;
  cout << "Maximum: " << bins.tail(1) << endl;
  
  return 0;

  // -- For some reason this is not working.
  mpliBegin();
  mpli("from pylab import *");
  // mpli("import matplotlib.pyplot as plt");
  // mpli("import numpy as np");
  mpliPrintSize();
  mpliExport(weights);
  mpliExport(bins);
  mpliExport(minval);
  mpliExport(maxval);
  mpliNamedExport("w", width);
  mpli("bar(bins, weights, width=w)");
  mpli("xlim(minval, maxval)");
  //mpli("ylim(-1, 1)");
  mpli("ylabel('Weights')");
  mpli("xlabel('Feature values')");
  mpliExport(output_path);
  mpli("draw()");
  mpli("savefig(output_path + '.png')");
  //mpli("savefig(output_path + '.pdf')");
  mpli("close()");

  return 0;
}

  
