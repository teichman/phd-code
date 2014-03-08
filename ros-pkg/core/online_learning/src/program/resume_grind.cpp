#include <matplotlib_interface/matplotlib_interface.h>
#include <boost/program_options.hpp>
#include <bag_of_tricks/bag_of_tricks.h>
#include <online_learning/active_learning_interface.h>
#include <online_learning/track_dataset_visualizer.h>
#include <online_learning/tbssl.h>

using namespace std;
using namespace Eigen;
namespace bpo = boost::program_options;
namespace bfs = boost::filesystem;

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  string output_dir;
  string unlabeled_td_dir;
  vector<string> test_paths;

  opts_desc.add_options()
    ("help,h", "produce help message")
    ("output-dir", bpo::value<string>(&output_dir)->required(), "Directory to put output.")
    ("unlabeled-td-dir", bpo::value<string>(&unlabeled_td_dir)->required(), "Directory of .td files to use as unlabeled data.")
    ("test", bpo::value< vector<string> >(&test_paths)->multitoken(), ".td files to test on periodically.")
    ;

  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << argv[0] << " [OPTS]" << endl; 
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  // -- Load the OnlineLearner.
  OnlineLearner learner((IfstreamWrapper(output_dir + "/learner.ol")));
  learner.setUnlabeledDir(unlabeled_td_dir);

  // -- Set test data.
  if(!test_paths.empty()) {
    TrackDataset::Ptr test = loadDatasets(test_paths);
    learner.setTestData(test);
  }

  // -- Go.
  ThreadPtr learning_thread = learner.launch();

  DGCTrackView view;
  VCMultiplexor multiplexor(&view);
  ActiveLearningViewController alvc(&multiplexor, NULL, &learner, unlabeled_td_dir);
  InductionViewController ivc(&learner, &multiplexor);
  multiplexor.addVC(&alvc);
  multiplexor.addVC(&ivc);

  ThreadPtr view_thread = view.launch();
  ThreadPtr alvc_thread = alvc.launch();
  ThreadPtr ivc_thread = ivc.launch();

  learning_thread->join();
  view_thread->join();
  alvc_thread->join();
  ivc_thread->join();
  
  return 0;
}
