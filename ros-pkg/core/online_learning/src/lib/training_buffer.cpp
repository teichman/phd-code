#include <online_learning/training_buffer.h>
#include <eigen_extensions/eigen_extensions.h>

using namespace std;
using namespace Eigen;

TrainingBuffer::TrainingBuffer() :
  max_frames_(-1),
  dataset_(new TrackDataset),
  swap_(new TrackDataset),
  total_tracks_(0),
  total_frames_(0)
{
}

void TrainingBuffer::init(size_t size)
{
  ROS_ASSERT(!nameMapping("cmap").empty());
  ROS_ASSERT(!nameMapping("dmap").empty());

  dataset_->tracks_.resize(size);
  for(size_t i = 0; i < dataset_->tracks_.size(); ++i)
    dataset_->tracks_[i] = Dataset::Ptr(new Dataset);
  dataset_->applyNameMappings(*this);
  
  swap_->tracks_.resize(size);
  for(size_t i = 0; i < swap_->tracks_.size(); ++i)
    swap_->tracks_[i] = Dataset::Ptr(new Dataset);
  swap_->applyNameMappings(*this);
}

void TrainingBuffer::merge(const TrackDataset& other)
{
  vector<int> indices(other.size());
  for(size_t i = 0; i < indices.size(); ++i)
    indices[i] = i;
  
  merge(other, indices);
}

void TrainingBuffer::merge(const TrackDataset& other, std::vector<int> oidx)
{
  ROS_ASSERT(dataset_->nameMappingsAreEqual(other));
  if(oidx.empty())
    return;

  // -- Copy in the first however many we can.
  size_t num_slots_left = (size_t)max((int)dataset_->size() - (int)total_tracks_, 0);
  size_t num_cp = min(oidx.size(), num_slots_left);

  for(size_t i = 0; i < num_cp; ++i) { 
    (*dataset_)[total_tracks_] = other[oidx[i]];
    // Apply the max_frames_ constraint, downsampling long tracks.
    if(max_frames_ > 0 && (int)(*dataset_)[total_tracks_].size() > max_frames_)
      downsampleTrack(&(*dataset_)[total_tracks_]);
    
    // Update statistics.
    ++total_tracks_;
    total_frames_ += other[oidx[i]].size();
    ROS_ASSERT(other[oidx[i]].size() > 0);
    int id = other[oidx[i]][0].label_.id();
    if(id >= 0)
      ++total_per_class_(id);
  }
  
  // -- Construct new indices and merge in any remainder using resampling.
  if(num_cp < oidx.size()) {
    oidx.erase(oidx.begin(), oidx.begin() + num_cp);
    mergeResampling(other, oidx);
  }
}

void TrainingBuffer::mergeResampling(const TrackDataset& other, const std::vector<int>& oidx)
{
  ROS_ASSERT(dataset_->nameMappingsAreEqual(other));
  if(oidx.empty())
    return;

  
  // -- Construct weights and index for low variance sampling.
  //    The other dataset is inserted at a random location to prevent
  //    bad behavior when this function is called repeatedly with a very
  //    small other dataset.

  vector<Sample> index(dataset_->size() + oidx.size());
  VectorXd weights(dataset_->size() + oidx.size());
  double buffer_weight = (double)total_tracks_ / ((double)dataset_->size() * ((double)total_tracks_ + oidx.size()));
  double other_weight = 1.0 / ((double)total_tracks_ + (double)oidx.size());
  int insertion = rand() % dataset_->size();
  int idx = 0;
  for(int i = 0; i < insertion; ++i, ++idx) {
    weights(idx) = buffer_weight;
    index[idx].dataset_ = dataset_.get();
    index[idx].idx_ = i;
  }
  for(size_t i = 0; i < oidx.size(); ++i, ++idx) {
    weights(idx) = other_weight;
    index[idx].dataset_ = &other;
    index[idx].idx_ = oidx[i];
  }
  for(size_t i = insertion; i < dataset_->size(); ++i, ++idx) {
    weights(idx) = buffer_weight;
    index[idx].dataset_ = dataset_.get();
    index[idx].idx_ = i;
  }
  ROS_ASSERT(idx == weights.rows());
  ROS_ASSERT(fabs(weights.sum() - 1.0) < 1e-6);

  // -- Run low variance sampling.
  VectorXi indices(dataset_->size());
  eigen_extensions::weightedSampleLowVariance(weights, &mersenne_, &indices);
    
  // -- Create the new training buffer.
  int num_other = 0;
  int num_buffer = 0;
  double total_buffer_weight = 0;
  double total_other_weight = 0;
  ROS_ASSERT(indices.rows() == (int)swap_->size());
  for(int i = 0; i < indices.rows(); ++i) {
    const Sample& sam = index[indices(i)];
    (*swap_)[i] = (*sam.dataset_)[sam.idx_];
    // Apply the max_frames_ constraint, downsampling long tracks.
    if(max_frames_ > 0 && (int)(*swap_)[i].size() > max_frames_)
      downsampleTrack(&(*swap_)[i]);

    if(sam.dataset_ == dataset_.get()) {
      ++num_buffer;
      total_buffer_weight += weights(i);
    }
    else {
      ++num_other;
      total_other_weight += weights(i);
    }
  }
  
  cout << "Merged dataset with " << other.size() << " tracks.  Given " << oidx.size() << " indices into this dataset." << endl;
  cout << "Total probability mass on buffer: " << buffer_weight * dataset_->size() << endl;
  cout << "Total probability mass on new (masked) chunk: " << other_weight * oidx.size() << endl;
  cout << "Total probability mass: " << other_weight * oidx.size() + buffer_weight * dataset_->size() << endl;
  ROS_ASSERT(fabs(other_weight * oidx.size() + buffer_weight * dataset_->size() - 1.0) < 1e-3);
  cout << "Mixed new buffer with " << num_buffer << " old tracks and " << num_other << " new tracks." << endl;

  // -- Swap it in, randomizing the order.
  vector<size_t> order;
  order.reserve(swap_->size());
  for(size_t i = 0; i < swap_->size(); ++i)
    order.push_back(i);
  random_shuffle(order.begin(), order.end());

  for(size_t i = 0; i < order.size(); ++i) {
    size_t idx = order[i];
    (*dataset_)[i] = (*swap_)[idx];
  }

  // -- Update statistics.
  total_tracks_ += oidx.size();
  for(size_t i = 0; i < oidx.size(); ++i)
    total_frames_ += other[oidx[i]].size();

  for(size_t i = 0; i < oidx.size(); ++i) {
    ROS_ASSERT(other[oidx[i]].size() > 0);
    int id = other[oidx[i]][0].label_.id();
    if(id >= 0)
      ++total_per_class_(id);
  }
}

void TrainingBuffer::downsampleTrack(Dataset* track) const
{
  ROS_ASSERT(max_frames_ > 0);
  ROS_ASSERT((int)track->size() > max_frames_);
  
  vector<int> indices(track->size());
  for(size_t i = 0; i < indices.size(); ++i)
    indices[i] = i;
  random_shuffle(indices.begin(), indices.end());

  vector<Instance> instances(max_frames_);
  for(size_t i = 0; i < instances.size(); ++i)
    instances[i] = (*track)[indices[i]];

  track->instances_ = instances;
}

size_t TrainingBuffer::numBytes() const
{
  return dataset_->numBytes() + swap_->numBytes();
}

std::string TrainingBuffer::status(const std::string& prefix) const
{
  ostringstream oss;
  oss << prefix << "TrainingBuffer memory usage: " << numBytes() << " bytes." << endl;
  oss << prefix << "Total tracks seen: " << total_tracks_ << endl;
  oss << prefix << "Total frames seen: " << total_frames_ << endl;
  oss << prefix << "Observed class proportions: " << total_per_class_.transpose() / total_tracks_ << endl;
  oss << prefix << "Buffer: " << endl;
  oss << dataset_->status(prefix + "  ", false);
  oss << prefix << "Swap: " << endl;
  oss << swap_->status(prefix + "  ", false);
  
  return oss.str();
}

void TrainingBuffer::_applyNameTranslator(const std::string& id, const NameTranslator& translator)
{
  if(id == "cmap") { 
    ROS_ASSERT(total_per_class_.rows() == (int)translator.oldSize());
    translator.translate(&total_per_class_);
  }
  dataset_->applyNameTranslator(id, translator);
  swap_->applyNameTranslator(id, translator);
}
  
void TrainingBuffer::serialize(std::ostream& out) const
{
  serializeNameMappings(out);
  out << *dataset_;
  eigen_extensions::serializeScalar(max_frames_, out);
  eigen_extensions::serializeScalar(total_tracks_, out);
  eigen_extensions::serializeScalar(total_frames_, out);
  eigen_extensions::serialize(total_per_class_, out);
}

void TrainingBuffer::deserialize(std::istream& in)
{
  deserializeNameMappings(in);
  in >> *dataset_;
  *swap_ = *dataset_;
  eigen_extensions::deserializeScalar(in, &max_frames_);
  eigen_extensions::deserializeScalar(in, &total_tracks_);
  eigen_extensions::deserializeScalar(in, &total_frames_);
  eigen_extensions::deserialize(in, &total_per_class_);
}

double TrainingBuffer::dilution() const
{
  if(total_tracks_ < dataset_->size())
    return total_tracks_ / dataset_->size();
  else
    return 1.0;
}
