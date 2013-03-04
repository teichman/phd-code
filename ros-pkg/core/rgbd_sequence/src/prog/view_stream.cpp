#include <boost/program_options.hpp>
#include <rgbd_sequence/stream_visualizer.h>
#include <rgbd_sequence/stream_sequence_base.h>

using namespace std;
using namespace rgbd;
using namespace pcl::visualization;
namespace bpo = boost::program_options;

int main(int argc, char** argv)
{
  string dir;
  bpo::options_description opts_desc("Allowed options");
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("seq", bpo::value<string>(&dir), "Sequence directory")
    ("only-stats", "Only print stats and exit")
    ;

  bpo::positional_options_description p;
  p.add("seq", 1);
  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  if(opts.count("help")) {
    cout << "Usage: view_stream SEQ [opts]" << endl << endl;
    cout << opts_desc << endl;
    return 1;
  }
  bpo::notify(opts);
  
  cout << "Looking at dir: " << dir << endl;
  StreamSequenceBase::Ptr sseq = StreamSequenceBase::initializeFromDirectory (dir);
  cout << "Loaded successfully" << endl;

  double mean_dt = 0;
  double max_dt = -std::numeric_limits<double>::max();
  for(size_t i = 1; i < sseq->size(); i++) {
    double dt = sseq->timestamps_[i] - sseq->timestamps_[i-1];
    mean_dt += dt;
    if(dt > max_dt)
      max_dt = dt;
  }
  mean_dt /= (double)sseq->size();
  cout << "--------------------" << endl;
  cout << "Mean fps: " << (double)sseq->size() / (sseq->timestamps_.back() - sseq->timestamps_.front()) << endl;
  cout << "Mean dt: " << mean_dt << endl;
  cout << "Max dt: " << max_dt << endl;
  cout << "Total time (s): " << sseq->timestamps_.back() - sseq->timestamps_.front() << endl;
  cout << "--------------------" << endl;
  cout << "Sensor model: " << endl;
  cout << sseq->model_.status("  ");
  
  if(opts.count("only-stats"))
    return 0;

  StreamVisualizer vis(sseq);
  vis.run();
  return 0;
}


