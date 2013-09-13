#include <Python.h>
#include <boost/program_options.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <online_learning/track_dataset_visualizer.h>

using namespace std;
using namespace Eigen;

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  opts_desc.add_options()
    ("help,h", "produce help message")
    ("td", bpo::value<string>()->required(), "TrackDataset augmented with PCDs.")
    ("output,o", bpo::value<string>(), "Where to save output to if you press 'S'.")
    ("save-test", bpo::value<string>())
    ;

  p.add("td", 1);

  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << argv[0] << " TD" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  TrackDataset::Ptr td(new TrackDataset);
  td->load(opts["td"].as<string>());
  if(opts.count("save-test")) {
    string path = opts["save-test"].as<string>();
    cout << "Saving to " << path << endl;
    td->save(path);
  }

  DGCTrackView view;
  TrackViewControllerBase tvc(&view);
  if(opts.count("output"))
    tvc.write_path_ = opts["output"].as<string>();
  else
    tvc.write_path_ = opts["td"].as<string>();
  cout << "On pressing 'S' this TD will be written to " << tvc.write_path_ << endl;
  
  tvc.setTrackDataset(td);
  ThreadPtr view_thread = view.launch();
  ThreadPtr tvc_thread = tvc.launch();
  view_thread->join();
  tvc_thread->join();

  return 0;
}
