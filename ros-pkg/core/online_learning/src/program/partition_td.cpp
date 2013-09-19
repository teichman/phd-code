#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <online_learning/grid_classifier.h>

using namespace std;
namespace bpo = boost::program_options;

size_t numBytesIncludingRaw(const Dataset& track) {
  size_t num = track.numBytes();
  for(size_t i = 0; i < track.size(); ++i) {
    PassthroughCustomSerializer::Data::Ptr data;
    data = boost::any_cast<PassthroughCustomSerializer::Data::Ptr>(track[i].raw_);
    num += data->data_.size();
  }

  return num;
}

size_t numBytesIncludingRaw(const TrackDataset& td) {
  size_t num = 0;
  for(size_t i = 0; i < td.size(); ++i)
    num += numBytesIncludingRaw(td[i]);
  return num;
}

void process(const std::string& td_path, const bpo::variables_map& opts)
{
  cout << "Loading " << td_path << "." << endl;
  TrackDataset td;
  td.load(td_path);

  vector<size_t> index(td.size());
  for(size_t i = 0; i < index.size(); ++i)
    index[i] = i;
  random_shuffle(index.begin(), index.end());

  if(opts.count("num-partitions")) {
    size_t num_partitions = opts["num-partitions"].as<size_t>();
    ROS_ASSERT(num_partitions > 1);
    ROS_ASSERT(num_partitions <= td.size());

    size_t total_tracks = 0;
    size_t total_frames = 0;
    for(size_t i = 0; i < num_partitions; ++i) {
      cout << "Generating partition " << i << " / " << num_partitions << endl;
      TrackDataset part;
      part.applyNameMappings(td);
      for(size_t j = i; j < td.size(); j += num_partitions)
        part.tracks_.push_back(td.tracks_[j]);

      ostringstream oss;
      oss << td_path.substr(0, td_path.size() - 3) << "-partition" << setw(2) << setfill('0') << i << ".td";
      HighResTimer hrt("Writing"); hrt.start();
      part.save(oss.str());
      hrt.stop(); cout << hrt.reportMilliseconds() << endl;
      cout << "Saved to " << oss.str() << "." << endl;
      total_tracks += part.size();
      total_frames += part.totalInstances();
    }
    ROS_ASSERT(total_tracks == td.size());
    ROS_ASSERT(total_frames == td.totalInstances());
  }
  else {
    ROS_ASSERT(opts.count("approx-gb"));
    double approx_gb = opts["approx-gb"].as<double>();
    TrackDataset::Ptr part(new TrackDataset);
    part->applyNameMappings(td);
    size_t idx = 0;
    for(size_t i = 0; i < td.size(); ++i) {
      part->tracks_.push_back(td.tracks_[i]);
      if(numBytesIncludingRaw(*part) + numBytesIncludingRaw(td[i]) > approx_gb * 1e9 ||
         i == td.size() - 1)
      {
        ostringstream oss;
        oss << td_path.substr(0, td_path.size() - 3) << "-partition" << setw(2) << setfill('0') << idx << ".td";
        cout << "Saving partition to " << oss.str() << endl;
        cout << "Expected file size is " << numBytesIncludingRaw(*part) << " bytes." << endl;
        part->save(oss.str());
        part = TrackDataset::Ptr(new TrackDataset);
        part->applyNameMappings(td);
        ++idx;
      }
    }
  }
}

int main(int argc, char** argv)
{
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  vector<string> td_paths;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("num-partitions,n", bpo::value<size_t>())
    ("approx-gb", bpo::value<double>(), "approx size in gb of the partition files")
    ("tds,d", bpo::value(&td_paths)->required())
    ;

  p.add("tds", -1);
  
  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }

  if((opts.count("num-partitions") && opts.count("approx-gb")) ||
     (!opts.count("num-partitions") && !opts.count("approx-gb")))
  {
    badargs = true;
  }
       
  if(opts.count("help") || badargs) {
    cout << "Usage: " << argv[0] << " [OPTS] TD [ TD ... ]" << endl;
    cout << "  You must specify either approx-gb or num-partitions." << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  for(size_t i = 0; i < td_paths.size(); ++i) {
    process(td_paths[i], opts);
  }  
  
  return 0;
}
