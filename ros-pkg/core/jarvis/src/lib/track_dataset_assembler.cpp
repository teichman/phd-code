#include <jarvis/track_dataset_assembler.h>

using namespace std;

TrackDatasetAssembler::TrackDatasetAssembler(std::string output_directory, size_t min_track_length,
                                             size_t max_track_length, size_t max_td_size) :
  output_directory_(output_directory),
  min_track_length_(min_track_length),
  max_track_length_(max_track_length),
  max_td_size_(max_td_size)
{
}

void TrackDatasetAssembler::update(const std::map<size_t, Blob::Ptr>& tracked_blobs)
{
  set<size_t> updated;

  // -- Update tracks_ with tracked_blobs contents.
  map<size_t, Blob::Ptr>::iterator bit;
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
      if(track.size() > min_track_length) {
        append(track);
        to_delete.push_back(id);
      }
    }

    // Tracks that are too long should be added to td_.
    else if(track.size() == max_track_length) {
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
  if(td_.totalInstances() >= max_num_instances) {
    saveAndClear();
  }
}


void TrackDatasetAssembler::append(const std::vector<Blob::Ptr>& track)
{
  
}

void TrackDatasetAssembler::saveAndClear()
{
  // -- Get next path name.
  
  
  // -- Save.
  td_.save(output_directory
}
