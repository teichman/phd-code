#ifndef TRACK_DATASET_ASSEMBLER_H
#define TRACK_DATASET_ASSEMBLER_H

#include <online_learning/dataset.h>
#include <jarvis/tracker.h>

//! Puts tracked Blobs into a TrackDataset.  Saves periodicall.
class TrackDatasetAssembler
{
public:
  //! Temporary storage while tracks accumulate.
  std::map<size_t, std::vector<Blob::Ptr> > tracks_;
  //! Tracks that meet the relevant criteria get put in here.
  TrackDataset td_;

  //! When td_.totalInstances() >= max_num_instances, td_ will be saved and cleared.
  TrackDatasetAssembler(std::string output_directory, size_t min_track_length,
                        size_t max_track_length, size_t max_num_instances);
  void update(const std::map<size_t, Blob::Ptr>& tracked_blobs);

protected:
  std::string output_directory_;
  size_t min_track_length_;
  size_t max_track_length_;
  size_t max_num_instances_;

  void append(const std::vector<Blob::Ptr>& track);
};

#endif // TRACK_DATASET_ASSEMBLER_H
