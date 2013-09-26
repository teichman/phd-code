#include <jarvis/track_dataset_assembler.h>
#include <bag_of_tricks/next_path.h>

using namespace std;

TrackDatasetAssembler::TrackDatasetAssembler(std::string output_directory, size_t min_track_length,
                                             size_t max_track_length, size_t max_num_instances) :
  output_directory_(output_directory),
  min_track_length_(min_track_length),
  max_track_length_(max_track_length),
  max_num_instances_(max_num_instances)
{
  // Give it an empty cmap and dmap.
  td_.applyNameMapping("cmap", NameMapping());
  td_.applyNameMapping("dmap", NameMapping());
}

void TrackDatasetAssembler::update(const std::map<size_t, Blob::Ptr>& tracked_blobs)
{
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
    ROS_ASSERT(track.size() <= max_track_length);
    
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
  dataset->applyNameMapping("cmap", NameMapping());
  dataset->applyNameMapping("dmap", NameMapping());

  dataset->instances_.resize(track.size());
  for(size_t i = 0; i < track.size(); ++i)
    dataset->instances_[i].raw_ = track[i];

  td_.tracks_.push_back(dataset);
}

