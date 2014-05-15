#include <online_learning/dataset.h>
#include <jarvis/tracker.h>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>

using namespace std;
namespace bfs = boost::filesystem;
namespace bpt = boost::posix_time;

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  string input;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("input", bpo::value(&input)->required(), "Input tds directory.");

  p.add("input", 1);
  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << argv[0] << " [OPTS] TD" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  bpt::ptime epoch(boost::gregorian::date(1970,1,1));
  bfs::directory_iterator it(input), eod;
  BOOST_FOREACH(bfs::path const & p, make_pair(it, eod)){
    TrackDataset td;
    td.load(p.string());
    const Dataset& lt = td[td.size()-1];
    ulong t = boost::any_cast<Blob::Ptr>(td[0][0].raw())->wall_timestamp_.toSec();
    cout << p.string() << endl;
    cout << "\t" << bpt::to_iso_string(bpt::from_time_t(t)) << endl;
    t = boost::any_cast<Blob::Ptr>(lt[lt.size()-1].raw())->wall_timestamp_.toSec();
    cout << "\t" << bpt::to_iso_string(bpt::from_time_t(t)) << endl;
  }
  
  return 0;
}
