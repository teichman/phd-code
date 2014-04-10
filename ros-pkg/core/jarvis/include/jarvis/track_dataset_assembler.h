#ifndef TRACK_DATASET_ASSEMBLER_H
#define TRACK_DATASET_ASSEMBLER_H

#include <online_learning/dataset.h>
#include <jarvis/tracker.h>

//! Puts tracked Blobs into a TrackDataset.  Saves periodicall.
class TrackDatasetAssembler
{
public:
  typedef boost::shared_ptr<TrackDatasetAssembler> Ptr;
  
  //! Temporary storage while tracks accumulate.
  std::map<size_t, std::vector<Blob::ConstPtr> > tracks_;
  //! Tracks that meet the relevant criteria get put in here.
  TrackDataset td_;

  //! When td_.totalInstances() >= max_num_instances, td_ will be saved and cleared.
  TrackDatasetAssembler(std::string output_directory, size_t min_track_length,
                        size_t max_track_length, size_t max_num_instances);
  //! Waits for serializer_ to finish.
  ~TrackDatasetAssembler();
  void update(const std::map<size_t, Blob::ConstPtr>& tracked_blobs);
  //! Saves any unsaved tracks in the TrackDatasetAssembler and clears the current contents.
  void flush();

protected:
  std::string output_directory_;
  size_t min_track_length_;
  size_t max_track_length_;
  size_t max_num_instances_;
  ThreadedSerializer<TrackDataset> serializer_;

  void append(const std::vector<Blob::ConstPtr>& track);
};

#endif // TRACK_DATASET_ASSEMBLER_H
