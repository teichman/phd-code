#ifndef TRAINING_BUFFER_H
#define TRAINING_BUFFER_H

#include <eigen_extensions/random.h>
#include <online_learning/dataset.h>

class TrainingBuffer : public NameMappable, public Serializable
{
public:
  typedef boost::shared_ptr<TrainingBuffer> Ptr;
  struct Sample
  {
    const TrackDataset* dataset_;
    int idx_;
  };

  int max_frames_;
  
  //! If a merged track has more than this number of frames,
  //! it will be downsampled.
  TrainingBuffer();
    
  //! Name mappings must be set before calling this.
  void init(size_t size);
  //! TODO: Need to let people change the labels, but not the content.
  TrackDataset::Ptr getTrackDataset() const { return dataset_; }
  void merge(const TrackDataset& other);
  //! oidx is a set of indices into other.
  void merge(const TrackDataset& other, std::vector<int> oidx);
  size_t totalTracks() const { return total_tracks_; }
  size_t totalFrames() const { return total_frames_; }
  std::string status(const std::string& prefix = "") const;
  std::string status(const TrackDataset& dataset, const std::string& prefix) const;
  size_t numBytes() const;
  void serialize(std::ostream& out) const;
  void deserialize(std::istream& in);
  //! If TB has seen fewer tracks than it stores in dataset_, sometimes you'll want to apply
  //! a weighting to represent that.
  double dilution() const;
  
protected:
  TrackDataset::Ptr dataset_;
  //! TODO: Use an algorithm that doesn't require copying the dataset.
  TrackDataset::Ptr swap_;
  double total_tracks_;
  double total_frames_;
  //! No background.
  Eigen::VectorXd total_per_class_;
  std::tr1::mt19937 mersenne_;

  TrainingBuffer(const TrainingBuffer& other);
  TrainingBuffer& operator=(const TrainingBuffer& other);

  void _applyNameTranslator(const std::string& id, const NameTranslator& translator);
  void downsampleTrack(Dataset* track) const;
  void mergeResampling(const TrackDataset& other, const std::vector<int>& oidx);
};

#endif // TRAINING_BUFFER_H
