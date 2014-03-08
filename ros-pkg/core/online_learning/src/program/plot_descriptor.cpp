#include <matplotlib_interface/matplotlib_interface.h>  // Must come first because of Python.
#include <boost/program_options.hpp>
#include <online_learning/tbssl.h>

using namespace std;
using namespace Eigen;
namespace bpo = boost::program_options;

void process(const TrackDataset& td, size_t did, size_t eid, size_t cid,
             vector<double>* pos, vector<double>* neg)
{
  pos->reserve(pos->size() + td.totalInstances());
  neg->reserve(neg->size() + td.totalInstances());
  for(size_t i = 0; i < td.size(); ++i) {
    const Dataset& track = td[i];
    for(size_t j = 0; j < track.size(); ++j) {
      const Instance& frame = track[j];
      if(!frame.descriptors_[did])
	continue;
      const VectorXf& descr = *frame.descriptors_[did];
      float val = descr[eid];
      if(frame.label_(cid) > 0)
	pos->push_back(val);
      else if(frame.label_(cid) < 0)
	neg->push_back(val);
    }
  }
}

int main(int argc, char** argv)
{
  // -- Parse args.
  size_t did, eid, cid;

  double xmin, xmax;
  int nbins;
  vector<string> td_paths;
  bpo::options_description opts_desc("Allowed options");
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("tds", bpo::value< vector<string> >(&td_paths)->required()->multitoken(), ".td files.")
    ("did", bpo::value<size_t>(&did)->required(), "Which descriptor.")
    ("eid", bpo::value<size_t>(&eid)->required(), "Which element of that descriptor.")
    ("cid", bpo::value<size_t>(&cid)->required(), "Which class.")
    ("xmin", bpo::value<double>(&xmin)->required())
    ("xmax", bpo::value<double>(&xmax)->required())
    ("nbins", bpo::value<int>(&nbins)->required())
    ;

  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).run(), opts);
  if(opts.count("help")) {
    cout << opts_desc << endl;
    return 1;
  }
  bpo::notify(opts);

  ostringstream oss;
  oss << "descr" << setw(2) << setfill('0') << did << "ele" << setw(4) << setfill('0') << eid << "class" << cid;
  string output_path = oss.str();
  cout << "Saving output to basename " << output_path << endl;

  vector<double> pos, neg;
  for(size_t i = 0; i < td_paths.size(); ++i) {
    cout << "Working on " << td_paths[i] << endl;
    TrackDataset td;
    td.load(td_paths[i]);
    process(td, did, eid, cid, &pos, &neg);
  }

  cout << "Positive examples, in sorted order: " << endl;
  sort(pos.begin(), pos.end());  // ascending.
  for(size_t i = 0; i < pos.size(); ++i)
    cout << pos[i] << endl;
  
  mpliBegin();
  mpli("import matplotlib.pyplot as plt");
  mpli("import numpy as np");
  mpliPrintSize();
  mpliExport(pos);
  mpliExport(neg);
  mpliExport(nbins);
  mpli("plt.hist([pos, neg], bins=nbins, histtype='barstacked', color=['red', 'gray'], label=['Positive', 'Negative'])");
  mpli("plt.xlabel('Feature value')");
  mpliExport(xmin);
  mpliExport(xmax);
  mpli("plt.xlim(xmin, xmax)");
  mpli("plt.ylabel('Num instances')");
  mpli("plt.legend()");
  mpliExport(output_path);
  mpli("plt.savefig(output_path + '.png')");
  mpli("plt.savefig(output_path + '.pdf')");
  
  return 0;
}
