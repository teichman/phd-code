#include <jarvis/track_dataset_assembler.h>
#include <bag_of_tricks/next_path.h>
#include <boost/filesystem.hpp>

using namespace std;
namespace bfs = boost::filesystem;

TrackDatasetAssembler::TrackDatasetAssembler(std::string output_directory, size_t min_track_length,
                                             size_t max_track_length, size_t max_num_instances) :
  output_directory_(output_directory),
  min_track_length_(min_track_length),
  max_track_length_(max_track_length),
  max_num_instances_(max_num_instances)
{
  // Give it an empty cmap and dmap.
  NameMapping cmap;
  cmap.addName("kitten");
  td_.applyNameMapping("cmap", cmap);
  td_.applyNameMapping("dmap", NameMapping());
  if(!bfs::exists(output_directory_))
    bfs::create_directory(output_directory_);
}

void TrackDatasetAssembler::update(const std::map<size_t, Blob::Ptr>& tracked_blobs)
{
  ScopedTimer st(__PRETTY_FUNCTION__);
  cout << "Got " << tracked_blobs.size() << " new blobs.  Have " << tracks_.size() << " in TDA." << endl;
  cout << "Total instances in td_: " << td_.totalInstances() << endl;
  set<size_t> updated;

  // -- Update tracks_ with tracked_blobs contents.
  map<size_t, Blob::Ptr>::const_iterator bit;
  for(bit = tracked_blobs.begin(); bit != tracked_blobs.end(); ++bit) {
    // If we have a track with this id, add the blob to that track.
    // Otherwise create a new track with this id and initialize it with just this blob.
    size_t id = bit->first;
    Blob::Ptr blob = bit->second;
    tracks_[id].push_back(blob);
    updated.insert(id);
  }

  // -- Check tracks_ for tracks that should be moved to td_.
  map<size_t, vector<Blob::Ptr> >::iterator it;
  vector<size_t> to_delete;
  for(it = tracks_.begin(); it != tracks_.end(); ++it) {
    size_t id = it->first;
    const vector<Blob::Ptr>& track = it->second;
    ROS_ASSERT(track.size() <= max_track_length_);
    
    // Tracks that didn't get updated should either be added to td_ or forgotten.
    if(!updated.count(id)) {
      to_delete.push_back(id);
      if(track.size() > min_track_length_)
        append(track);
    }

    // Tracks that are too long should be added to td_.
    else if(track.size() == max_track_length_) {
      append(track);
      to_delete.push_back(id);
    }
  }

  // -- Delete unneeded tracks.
  for(size_t i = 0; i < to_delete.size(); ++i) {
    ROS_ASSERT(tracks_.find(to_delete[i]) != tracks_.end());
    tracks_.erase(to_delete[i]);
  }
  to_delete.clear();
  
  // -- If td_ is too big, save it and start a new one.
  if(td_.totalInstances() >= max_num_instances_) {
    string path = nextPath(output_directory_, "jarvis-", ".td", 4);
    td_.save(path);
    td_.tracks_.clear();
  }
}


void TrackDatasetAssembler::append(const std::vector<Blob::Ptr>& track)
{
  Dataset::Ptr dataset(new Dataset);
  
  dataset->instances_.resize(track.size());
  for(size_t i = 0; i < track.size(); ++i)
    dataset->instances_[i].raw_ = track[i];

  NameMapping cmap;
  cmap.addName("kitten");
  dataset->applyNameMapping("cmap", cmap);
  dataset->applyNameMapping("dmap", NameMapping());

  td_.tracks_.push_back(dataset);
}

void TrackDatasetAssembler::flush()
{
  // -- Move any valid tracks to td_, then clear tracks_.
  map<size_t, vector<Blob::Ptr> >::iterator it;
  for(it = tracks_.begin(); it != tracks_.end(); ++it) {
    const vector<Blob::Ptr>& track = it->second;
    ROS_ASSERT(track.size() <= max_track_length_);
    if(track.size() > min_track_length_)
      append(track);
  }
  tracks_.clear();
  
  // -- Save and clear td_.
  string path = nextPath(output_directory_, "jarvis-", ".td", 4);
  td_.save(path);
  td_.tracks_.clear();
}
