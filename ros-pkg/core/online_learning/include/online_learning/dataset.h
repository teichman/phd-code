#ifndef DATASET_H
#define DATASET_H

#include <vector>
#include <boost/any.hpp>
#include <boost/shared_ptr.hpp>
#include <Eigen/Eigen>
#include <name_mapping/name_mapping.h>
#include <serializable/serializable.h>

class CustomSerializer;

// Represents a reference to where additional boost::any data is available
// in external storage. To be used for on-demand loading.
struct StorageReference {
  StorageReference() :
    td_filename_(""), file_offset_(0) {}

  std::string td_filename_;
  uint64_t file_offset_;

  // Whether StorageReference can be used to load data from disk or not.
  // raw_.empty() is used to determine if the raw data *has* been loaded from disk.
  bool valid() const { return (td_filename_ != ""); }
};


//! Class that represents human-generated annotations in {-1, 0, +1}^n
//! and classifier-generated predictions (log odds vectors) in \R^n.
class Label : public Eigen::VectorXf, public NameTranslatable, public Serializable
{
public:  
  //! Returns the max-confidence id.  If there are ties, you get a random one of them.
  //! This could be very bad if you have multiply-labeled objects and try to get the id
  //! of an annotation.
  //! Returns -1 for "none of the known classes" and -2 for "unlabeled".
  int id(float* confidence = NULL) const; // __attribute__((deprecated("Label::id() assumes that an object can only have one positive classification.  This is not generally correct!  Think \"car\" vs \"police_car\".  Don't use this function...")));
  
  void serialize(std::ostream& out) const;
  void deserialize(std::istream& in);
  std::string status(const NameMapping& cmap, const std::string& prefix = "") const;

  //! Exact equality of floats.
  bool operator==(const Label& other) const;
  bool operator!=(const Label& other) const { return !(*this == other); }

  // -- Pass-through Eigen operators.
  // This nasty thing is required to initialize from VectorXf::Zero(.), etc.
  typedef Eigen::CwiseNullaryOp<Eigen::internal::scalar_constant_op<float>,
				Eigen::Matrix<float, Eigen::Dynamic, 1> > EigTemp;

  Label() {}
  Label(const EigTemp& other) { Eigen::VectorXf::operator=(other); }
  Label(const Eigen::VectorXf& other) { Eigen::VectorXf::operator=(other); }
  Label(const std::vector<float>& other) { *this = Eigen::VectorXf(other.size()); for(int i = 0; i < rows(); ++i) coeffRef(i) = other[i]; }
  Label& operator=(const Eigen::VectorXf& other) { Eigen::VectorXf::operator=(other); return *this; }
  Label& operator=(const EigTemp other) { Eigen::VectorXf::operator=(other); return *this; }
  Label& operator+=(const Eigen::VectorXf& other) { Eigen::VectorXf::operator+=(other); return *this; }
  Label& operator-=(const Eigen::VectorXf& other) { Eigen::VectorXf::operator-=(other); return *this; }
  Label& operator*=(float coeff) { Eigen::VectorXf::operator*=(coeff); return *this; }
  Label& operator/=(float coeff) { Eigen::VectorXf::operator/=(coeff); return *this; }
  operator Eigen::VectorXf() const { return *this; }
  //! Converts log odds estimates in R to {-1, 0, +1} labels based on the sign of the log odds.
  //! Returns a copy.
  Label sign() const;
  //! Converts log odds estimates in R to {-1, 0, +1} labels based on the given thresholds.
  //! Changes the contents of this object.
  Label& threshold(const Eigen::VectorXf& upper_thresholds,
		   const Eigen::VectorXf& lower_thresholds);
  //! Handy for assigning a Label to a field of a ROS message.
  std::vector<float> vector() const;
  
protected:
  //! "cmap" only.
  void _applyNameTranslator(const std::string& id, const NameTranslator& translator);
};

//! Represents a training or test example with associated descriptors.
//! Does not store its NameMapping, but has one implicitly associated with it.
//! cmap is the class map.
//! dmap is the descriptor map.  It is assumed the dmap includes a common name
//!   and a uniquely-identifying hash.
class Instance : public NameTranslatable, public Serializable
{
public:
  // No Ptr or ConstPtr.  NameTranslatable but not NameMappable objects should usually
  // be copied rather than have pointers; otherwise it may not be clear what NameMapping
  // applies to them.
  
  //! Usually the label is a human-generated annotation in {-1, 0, +1}^n,
  //! but that might not always be the case, for example in a soft EM algorithm.
  Label label_;
  //! NULL indicates a missing descriptor.
  //! Instance owns the descriptors and will deallocate them on destruction.
  std::vector<Eigen::VectorXf*> descriptors_;
  //! Custom data.  If custom_serializer_ is set
  //! to something other than EmptyCustomSerializer,
  //! this will get serialized and deserialized
  //! with the instance.  Otherwise it will be ignored.
  //! TODO: It's possible that raw_ should be actual data, i.e. something
  //! with a copy constructor that will make a deep copy.
  //! Otherwise the various clone() functions will not work
  //! as expected.  This could lead to immense pain.
  //! Instances are meant to be deep copied anyway...
  static boost::shared_ptr<CustomSerializer> custom_serializer_;

  Instance() {}
  //! Deep-copies the descriptors.
  Instance(const Instance& other);
  ~Instance();
  //! Deep-copies the descriptors, shallow-copies raw_.
  Instance& operator=(const Instance& other);
  //! Deep-copies.
  void copy(const std::vector<Eigen::VectorXf*>& descriptors);
  //! Deep-copies.
  void copy(const std::vector<const Eigen::VectorXf*>& descriptors);
  //! Deletes all descriptors and replaces them with null pointers.
  //! Does not clear descriptors_.
  void deleteDescriptors();
  bool operator==(const Instance& other) const;
  bool operator!=(const Instance& other) const { return !(*this == other); }
  Eigen::VectorXf* operator[](size_t i) const { return descriptors_[i]; }
  Eigen::VectorXf*& operator[](size_t i) { return descriptors_[i]; }
  size_t size() const { return descriptors_.size(); }
  bool empty() const { return descriptors_.empty(); }
  //! Gross and temporary hack.  Sum of the middle elements.
  double hash() const;
  
  void serialize(std::ostream& out) const;
  void deserialize(std::istream& in) { deserialize(in, ""); }
  void deserialize(std::istream& in, const std::string &filename);
  std::string status(const std::string& prefix = "") const;
  //! This does not include the size of the raw_ data.
  size_t numBytes() const;
  
  // Load raw on demand from disk when necessary
  boost::any& raw() const;
  template<typename ValueType>
  void setRaw(const ValueType &data) { raw_ = data; }
  void clearRaw();
  void makeLightweight(const std::string& td_path);

protected:
  mutable StorageReference raw_ref_;
  mutable boost::any raw_;
  //! Can apply "dmap" or "cmap".
  void _applyNameTranslator(const std::string& id, const NameTranslator& translator);
};

//! Since descriptors may be missing, it's sometimes necessary to have a
//! structure describing the dimensionality of all descriptor spaces.
class DescriptorDimensionality : public NameTranslatable, public Serializable
{
public:
  Eigen::VectorXi num_elements_;

  DescriptorDimensionality() {}
  DescriptorDimensionality(size_t num_descriptors) : num_elements_(Eigen::VectorXi::Ones(num_descriptors) * -1) {}
  int operator[](size_t i) const { return num_elements_(i); }
  int& operator[](size_t i) { return num_elements_.coeffRef(i); }
  bool operator==(const DescriptorDimensionality& other) const;
  bool operator!=(const DescriptorDimensionality& other) const { return !(*this == other); }
  size_t size() const { return num_elements_.rows(); }
  bool empty() const { return (num_elements_.rows() == 0); }
  int total() const;
  
  void serialize(std::ostream& out) const;
  void deserialize(std::istream& in);
  
protected:
  void _applyNameTranslator(const std::string& id, const NameTranslator& translator);
};

//! Represents a set of instances, all with the same NameMapping.
class Dataset : public NameMappable, public Serializable
{
public:
  typedef boost::shared_ptr<Dataset> Ptr;
  typedef boost::shared_ptr<const Dataset> ConstPtr;
  
  std::vector<Instance> instances_;

  void serialize(std::ostream& out) const;
  void deserialize(std::istream& in) { deserialize(in, ""); }
  void deserialize(std::istream& in, const std::string &filename);
  std::string status(const std::string& prefix = "", bool show_name_mappings = true) const;
  void deleteDescriptors();
  //! This does not include the size of the raw_ data.
  size_t numBytes() const;
  size_t size() const { return instances_.size(); }
  bool empty() const { return instances_.empty(); }
  //! All labels of the dataset must be the same and
  //! instances_ must not be empty.
  const Label& label() const;
  //! Sets all instances to have the same label.
  void setLabel(const Label& label);
  double hash() const;
  //! Deep copy
  Dataset::Ptr clone() const;
  
  Dataset& operator+=(const Dataset& other);
  //! Does *not* include the custom data, because that can be anything.
  bool operator==(const Dataset& other) const;
  bool operator!=(const Dataset& other) const { return !(*this == other); }
  const Instance& operator[](size_t i) const { return instances_[i]; }
  Instance& operator[](size_t i) { return instances_[i]; }
  //! Sets the magnitude of the label values, interpreted as importance by all Trainer objects.
  void setImportance(float importance);

  void clearRaw();
  void makeLightweight(const std::string& td_path);
      
protected:
  //! Can apply "dmap" or "cmap".
  void _applyNameTranslator(const std::string& id, const NameTranslator& translator);
};

//! Represents a set of tracks, each of which is contained in its own Dataset.
//!  
//! Note on name mappings: tracks_ contains shared_ptrs which can cause you serious
//! pain if you don't know what you are doing.  The pitfall is to copy a Dataset::Ptr
//! into two different TrackDatasets, then apply a new name mapping to one of them.
//! Disaster will result, as the other TrackDataset will have the old name mapping
//! while one of its contained Datasets will have a different one.
//!
//! The benefits of using shared_ptrs here outweigh the risks, but be careful.
class TrackDataset : public NameMappable, public Serializable
{
public:
  typedef boost::shared_ptr<TrackDataset> Ptr;
  typedef boost::shared_ptr<const TrackDataset> ConstPtr;

  std::vector<Dataset::Ptr> tracks_;

  TrackDataset() {}
  TrackDataset(std::istream& in) { deserialize(in); }
  ~TrackDataset();
  void serialize(std::ostream& out) const;
  void deserialize(std::istream& in) { return deserialize(in, ""); }
  void deserialize(std::istream& in, const std::string& filename);
  //! Makes a deep copy of all the data.
  //! Watch out!  This does NOT necessarily clone the raw_ data.
  //! raw_ will be copied, whatever it is... and it's probably a shared_ptr.
  //! That will be a shallow pointer.
  TrackDataset::Ptr clone() const;
  bool operator==(const TrackDataset& other) const;
  TrackDataset& operator+=(const TrackDataset& other);
  bool operator!=(const TrackDataset& other) const { return !(*this == other); }
  //! Note the dereference.  You can do td[trid][frid][did](eid).
  //! track, frame, descriptor, element.
  const Dataset& operator[](size_t i) const { return *tracks_[i]; }
  Dataset& operator[](size_t i) { return *tracks_[i]; }
  void deleteDescriptors();
  //! Deep-copies track i.
  Dataset::Ptr copy(size_t i) { return Dataset::Ptr(new Dataset(*tracks_[i])); }
 
  //! Overridden to print allocation and deallocation statements.
  void load(const std::string& filename);
  std::vector<Label> labels() const;
  
  const Label& label(size_t i) const;
  std::string status(const std::string& prefix = "", bool show_name_mappings = false) const;
  //! This does not include the size of the raw_ data.
  size_t numBytes() const;
  size_t size() const { return tracks_.size(); }
  bool empty() const { return tracks_.empty(); }
  size_t totalInstances() const;
  //! Sets the magnitude of the label values, interpreted as importance by all Trainer objects.
  void setImportance(float importance);
  //! Tries to determine the DescriptorDimensionality object.
  //! Crashes if it can't.  (Should probably instead return a special value for "unknown".)
  DescriptorDimensionality inferDescriptorDimensionality() const;
  //! Returns num_pos / num_neg for class_name.
  double labelRatio(const std::string& class_name) const;
  
  void clearRaw();
  void makeLightweight(const std::string& td_path);

protected:
  //! Can apply "dmap" or "cmap".
  void _applyNameTranslator(const std::string& id, const NameTranslator& translator);

  // Convenience functions used internally.
  // Users of TrackDataset should use operator[].
  const Dataset& track(size_t i) const { return *tracks_[i]; }
  Dataset& track(size_t i) { return *tracks_[i]; }
};

TrackDataset::Ptr loadDatasets(const std::vector<std::string> paths,
                               const NameMapping& cmap = NameMapping(),
                               bool verbose = false);
//! Any tracks longer than max_length get cut to the max length.
void cropTracks(size_t max_length, TrackDataset* dataset);
//! Any tracks longer than max_length get broken into smaller tracks.
void splitTracks(size_t max_length, TrackDataset* dataset);
//! Any tracks longer than length get broken into smaller tracks.
//! Any tracks smaller than length are dropped. 
void splitTracksFixedLength(size_t length, TrackDataset* dataset);
//! -1, 0, +1
float sign(float val);
//! Saves datasets to basepath-CLASS-{pos,neg}.td.
void saveByClassAndLabel(const TrackDataset& td, const std::string& basepath);
NameMapping getStubDescriptorMap(int num_descriptors);
//! Depends on the hash functionality.  Could be brittle depending on your case.
//! This will fail if you don't have descriptors, for example.
void removeDuplicates(TrackDataset* td);
//! Divides tracks between two different datasets.
//! Does not make deep copies.
void splitDataset(const TrackDataset& td, double pct0, TrackDataset* split0, TrackDataset* split1);
//! Samples num_tracks from td, considering only tracks with non-zero label for class_name.
//! The ratio of positive to negative of class_name will be set to ratio.
//! Does not make deep copies.
TrackDataset sampleDatasetProportional(const TrackDataset& td, size_t num_tracks,
                                       const std::string& class_name, double ratio);

#endif // DATASET_H
