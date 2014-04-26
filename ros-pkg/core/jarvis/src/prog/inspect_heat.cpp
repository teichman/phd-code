#include <jarvis/tracker.h>
#include <boost/program_options.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace Eigen;

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  string td_path;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("td", bpo::value(&td_path)->required(), "TrackDataset.")
    ;

  p.add("td", 1);

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

  TrackDataset td;
  td.load(td_path);

  for(size_t i = 0; i < td.size(); ++i) {
    const Dataset& track = td[i];
    for(size_t j = 0; j < track.size(); ++j) {
      cout << "Track: " << i << " Frame: " << j;
      const Blob& blob = *boost::any_cast<Blob::ConstPtr>(track[j].raw());
      cout << " Timestamp: " << fixed << setprecision(16) << setw(16) << setfill('0') << blob.wall_timestamp_.toSec() << endl;

      cv::Mat3b img = blob.image();
      cv::imshow("Image", img);
      cv::waitKey(5);
    }
  }

  return 0;
}

