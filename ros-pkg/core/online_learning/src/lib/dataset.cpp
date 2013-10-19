#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <online_learning/dataset.h>

using namespace Eigen;
using namespace std;

typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<pcl::PointXYZRGB> Cloud;

void serializePCD(const Cloud& pcd, std::ostream& out)
{
  eigen_extensions::serializeScalar(pcd.header.stamp * 1e-6, out);
  eigen_extensions::serializeScalar(pcd.size(), out);
  Matrix<uint8_t, 3, 1> rgb;
  Vector3f xyz;
  for(size_t i = 0; i < pcd.size(); ++i) {
    xyz = pcd[i].getVector3fMap();
    eigen_extensions::serialize(xyz, out);
    rgb.coeffRef(0) = pcd[i].r;
    rgb.coeffRef(1) = pcd[i].g;
    rgb.coeffRef(2) = pcd[i].b;
    eigen_extensions::serialize(rgb, out);
  }
}

void deserializePCD2(std::istream& in, Cloud* pcd)
{
  double timestamp;
  eigen_extensions::deserializeScalar(in, &timestamp);
  pcd->header.stamp = timestamp * 1e6;
  size_t num_pts;
  eigen_extensions::deserializeScalar(in, &num_pts);

  size_t num_bytes_per_point = 3 * (sizeof(uint8_t) + sizeof(float));
  num_bytes_per_point += 6 * sizeof(int);  // eigen_extensions headers
  size_t total_num_bytes = num_pts * num_bytes_per_point;
  uint8_t data[total_num_bytes];
  in.read((char*)data, total_num_bytes);
  
  pcd->clear();
  pcd->resize(num_pts);
  for(size_t i = 0; i < pcd->size(); ++i) {
    Point& pt = pcd->at(i);
    int idx = i * num_bytes_per_point + 3 * sizeof(int);
    pt.x = *(float*)(data + idx + 0 * sizeof(float));
    pt.y = *(float*)(data + idx + 1 * sizeof(float));
    pt.z = *(float*)(data + idx + 2 * sizeof(float));
    idx += 3 * sizeof(float) + 3 * sizeof(int);
    pt.r = *(char*)(data + idx + 0 * sizeof(uint8_t));
    pt.g = *(char*)(data + idx + 1 * sizeof(uint8_t));
    pt.b = *(char*)(data + idx + 2 * sizeof(uint8_t));
  }

  pcd->height = 1;
  pcd->width = pcd->size();
}

void deserializePCD(std::istream& in, Cloud* pcd)
{
  double timestamp;
  eigen_extensions::deserializeScalar(in, &timestamp);
  pcd->header.stamp = timestamp * 1e6;
  
  size_t num_pts;
  eigen_extensions::deserializeScalar(in, &num_pts);
  pcd->clear();
  pcd->resize(num_pts);
  Matrix<uint8_t, 3, 1> rgb;
  Vector3f xyz;
  for(size_t i = 0; i < pcd->size(); ++i) {
    Point& pt = pcd->at(i);
    eigen_extensions::deserialize(in, &xyz);
    pt.getVector3fMap() = xyz;  // This is as fast as doing the assignment manually.
    eigen_extensions::deserialize(in, &rgb);
    pt.r = rgb.coeffRef(0);
    pt.g = rgb.coeffRef(1);
    pt.b = rgb.coeffRef(2);
  }

  pcd->height = 1;
  pcd->width = pcd->size();
}

void PCDSerializer::serialize(const boost::any& raw, std::ostream& out) const
{
  Cloud::Ptr pcd = boost::any_cast<Cloud::Ptr>(raw);
  serializePCD(*pcd, out);
}

void PCDSerializer::deserialize(std::istream& in, boost::any* raw) const
{
  Cloud::Ptr pcd(new Cloud);
  deserializePCD2(in, pcd.get());
  *raw = pcd;
}

void PassthroughCustomSerializer::serialize(const boost::any& raw, std::ostream& out) const
{
  Data::Ptr data = boost::any_cast<Data::Ptr>(raw);
  out << data->name_ << endl;
  eigen_extensions::serializeScalar(data->data_.size(), out);
  if(!data->data_.empty())
    out.write((char*)&data->data_[0], data->data_.size());
}

void PassthroughCustomSerializer::deserialize(std::string original_name,
                                              size_t num_bytes,
                                              std::istream& in,
                                              boost::any* raw) const
{
  Data::Ptr data(new Data);
  data->name_ = original_name;
  data->data_.resize(num_bytes);
  if(num_bytes > 0)
    in.read((char*)&data->data_[0], num_bytes);

  *raw = data;
}


/************************************************************
 * Label
 ************************************************************/

int Label::id(float* confidence) const
{
  ROS_WARN_ONCE("id() is nasty.  Failure modes: multiply-labeled objects, bg (-1) label doesn't necessarily mean that it's a negative example for all known classes, but other code will probably interpret it as such.");

  int id;
  float max = maxCoeff(&id);
  if(max <= 0) {
    id = -1;
    if(confidence)
      *confidence = -max;
  }
  else if(confidence)
    *confidence = max;
  
  // Edge case: unlabeled examples.
  if(max == 0) {
    bool unlabeled = true;
    for(int i = 0; i < rows() && unlabeled; ++i) 
      if(coeffRef(i) != 0)
	unlabeled = false;

    if(unlabeled)
      return -2;
  }
     
  return id;
}

void Label::serialize(std::ostream& out) const
{
  eigen_extensions::serialize(*this, out);
}

void Label::deserialize(std::istream& in)
{
  eigen_extensions::deserialize(in, this);
}

bool Label::operator==(const Label& other) const
{
  if(rows() != other.rows())
    return false;

  for(int i = 0; i < rows(); ++i)
    if(this->coeffRef(i) != other(i))
      return false;

  return true;
}

void Label::_applyNameTranslator(const std::string& id, const NameTranslator& translator)
{
  translator.translate(this);
}

std::string Label::status(const NameMapping& cmap, const std::string& prefix) const
{
  ROS_ASSERT((int)cmap.size() == rows());
  ostringstream oss;
  for(int i = 0; i < rows(); ++i)
    oss << prefix << cmap.toName(i) << ": " << coeffRef(i) << endl;
  return oss.str();
}

Label Label::sign() const
{
  Label copy = *this;
  for(int i = 0; i < copy.rows(); ++i)
    copy(i) = ::sign((float)copy.coeffRef(i));  // Yuck.
  return copy;
}

Label& Label::threshold(const Eigen::VectorXf& upper_thresholds,
			const Eigen::VectorXf& lower_thresholds)
{
  for(int i = 0; i < rows(); ++i) {
    ROS_ASSERT(upper_thresholds(i) > lower_thresholds(i));
    if(coeffRef(i) > upper_thresholds(i))
      coeffRef(i) = 1;
    else if(coeffRef(i) < lower_thresholds(i))
      coeffRef(i) = -1;
    else
      coeffRef(i) = 0;
  }
  return *this;
}


/************************************************************
 * Instance
 ************************************************************/

Instance::Instance(const Instance& other)
{
  *this = other;
}

Instance& Instance::operator=(const Instance& other)
{
  if(&other == this)
    return *this;

  label_ = other.label_;
  raw_ = other.raw_;

  copy(other.descriptors_);
  return *this;
}

void Instance::deleteDescriptors()
{
  for(size_t i = 0; i < descriptors_.size(); ++i) {
    if(descriptors_[i]) {
      delete descriptors_[i];
      descriptors_[i] = NULL;
    }
  }
}

void TrackDataset::deleteDescriptors()
{
  for(size_t i = 0; i < tracks_.size(); ++i)
    tracks_[i]->deleteDescriptors();
}

void Instance::copy(const std::vector<Eigen::VectorXf*>& descriptors)
{
  vector<const Eigen::VectorXf*> cds(descriptors.size());
  for(size_t i = 0; i < descriptors.size(); ++i)
    cds[i] = descriptors[i];

  copy(cds);
}

void Instance::copy(const std::vector<const Eigen::VectorXf*>& descriptors)
{
  deleteDescriptors();
  descriptors_.clear();
  descriptors_.resize(descriptors.size(), NULL);
  for(size_t i = 0; i < descriptors_.size(); ++i)
    if(descriptors[i])
      descriptors_[i] = new VectorXf(*descriptors[i]);
}

Instance::~Instance()
{
  deleteDescriptors();
}

void Instance::serialize(std::ostream& out) const
{
  out << label_;
  
  int buf = descriptors_.size();
  out.write((char*)&buf, sizeof(buf));
  for(size_t i = 0; i < descriptors_.size(); ++i) {
    if(!descriptors_[i]) {
      buf = 0;
      out.write((char*)&buf, sizeof(buf));
    }
    else {
      buf = 1;
      out.write((char*)&buf, sizeof(buf));
      eigen_extensions::serialize(*descriptors_[i], out);
    }
  }

  ROS_ASSERT(custom_serializer_);

  if(custom_serializer_->name() == "PassthroughCustomSerializer") {
    // Writes the original name, num bytes, and the binary blob.
    custom_serializer_->serialize(raw_, out);
  }
  else {
    out << Instance::custom_serializer_->name() << endl;

    // -- Write the number of bytes followed by the serialized raw data.
    long num_bytes_pos = out.tellp();
    ROS_ASSERT(num_bytes_pos != -1);
    eigen_extensions::serializeScalar((size_t)0, out);  // placeholder
    long start = out.tellp();
    ROS_ASSERT(start != -1);
    Instance::custom_serializer_->serialize(raw_, out);
    long end = out.tellp();
    ROS_ASSERT(end != -1);
    long num_bytes = end - start;
    ROS_ASSERT(num_bytes >= 0);
    out.seekp(num_bytes_pos);
    eigen_extensions::serializeScalar((size_t)num_bytes, out);
    out.seekp(end);
  }
}

void Instance::deserialize(std::istream& in)
{
  in >> label_;
  int buf;
  in.read((char*)&buf, sizeof(buf));
  descriptors_.resize(buf, NULL);
  for(size_t i = 0; i < descriptors_.size(); ++i) {
    in.read((char*)&buf, sizeof(buf));
    if(buf == 1) {
      descriptors_[i] = new Eigen::VectorXf;
      eigen_extensions::deserialize(in, descriptors_[i]);
    }
  }

  // -- Custom data handling.  A custom serializer must be set.
  // First read the metadata.
  ROS_ASSERT(Instance::custom_serializer_);
  string custom_serializer_name;
  getline(in, custom_serializer_name);
  size_t num_bytes;
  eigen_extensions::deserializeScalar(in, &num_bytes);

  if(custom_serializer_->name() == "EmptyCustomSerializer") {
    // Skip the raw data.
    // TODO: Should this instead be an assertion fail?
    in.seekg(num_bytes, ios_base::cur);
  }
  else if(custom_serializer_->name() == "PassthroughCustomSerializer") {
    // Load the raw data as a binary blob with the original name.
    PassthroughCustomSerializer::Ptr pcs = boost::dynamic_pointer_cast<PassthroughCustomSerializer>(custom_serializer_);
    ROS_ASSERT(pcs);
    pcs->deserialize(custom_serializer_name, num_bytes, in, &raw_);
  }
  else {
    // Otherwise the names should match up.
    ROS_ASSERT(custom_serializer_name == Instance::custom_serializer_->name());
    Instance::custom_serializer_->deserialize(in, &raw_);
  }
}

bool Instance::operator==(const Instance& other) const
{
  if(label_ != other.label_)
    return false;
  if(descriptors_.size() != other.descriptors_.size())
    return false;
  for(size_t i = 0; i < descriptors_.size(); ++i) {
    if(!descriptors_[i] && !other.descriptors_[i])
      continue;
    if(!descriptors_[i] && other.descriptors_[i])
      return false;
    if(descriptors_[i] && !other.descriptors_[i])
      return false;
    if(descriptors_[i]->rows() != other.descriptors_[i]->rows())
      return false;
    for(int j = 0; j < descriptors_[i]->rows(); ++j)
      if(descriptors_[i]->coeffRef(j) != other.descriptors_[i]->coeffRef(j))
	return false;
  }

  return true;
}

void Instance::_applyNameTranslator(const std::string& id, const NameTranslator& translator)
{
  if(id == "cmap") {
    label_.applyNameTranslator(id, translator);
  }
  else if(id == "dmap") {
    // TODO: It might be better to use shared_ptrs here, or perhaps
    // VectorXfs that are simply of zero length to indicate a missing
    // descriptor.  As it is we have to do some nastly explicit handling.
    for(size_t i = 0; i < descriptors_.size(); ++i) {
      int idx = translator.toNew(i);
      if(idx == NameTranslator::NO_ID) {
        delete descriptors_[i];
        descriptors_[i] = NULL;
      }
    }
    translator.translate(&descriptors_, (VectorXf*)NULL);
  }
  else
    ROS_ASSERT(0);
}

size_t Instance::numBytes() const
{
  size_t num = 0;
  for(size_t i = 0; i < descriptors_.size(); ++i)
    if(descriptors_[i])
      num += descriptors_[i]->rows() * sizeof(float);

  num += label_.rows() * sizeof(float);
  return num;
}

std::string Instance::status(const std::string& prefix) const 
{
  ostringstream oss;
  oss << prefix << "Instance memory usage: " << numBytes() << " bytes." << endl;
  oss << prefix << "Label: " << label_.transpose() << endl;
  oss << prefix << descriptors_.size() << " descriptors: " << endl;
  for(size_t i = 0; i < descriptors_.size(); ++i) {
    oss << prefix << "  " << i << ":  ";
    if(!descriptors_[i])
      oss << "missing";
    else
      oss << descriptors_[i]->transpose();
    oss << endl;
  }
  return oss.str();
}

double Instance::hash() const
{
  double val = 0;
  for(size_t i = 0; i < descriptors_.size(); ++i) {
    //cout << i << ": " << descriptors_[i] << "." << endl;
    if(!descriptors_[i] || descriptors_[i]->rows() == 0)
      --val;
    else
      val += descriptors_[i]->coeffRef(descriptors_[i]->rows() / 2);
  }
  return val;
}

/************************************************************
 * DescriptorDimensionality
 ************************************************************/

int DescriptorDimensionality::total() const
{
  int total = 0;
  for(int i = 0; i < num_elements_.rows(); ++i) {
    ROS_ASSERT(num_elements_[i] > 0);  // Make sure they have all been initialized.
    total += num_elements_[i];
  }
  return total;
}

bool DescriptorDimensionality::operator==(const DescriptorDimensionality& other) const
{
  if(other.num_elements_.rows() != num_elements_.rows())
    return false;

  for(int i = 0; i < num_elements_.rows(); ++i)
    if(num_elements_[i] != other.num_elements_[i])
      return false;

  return true;
}

void DescriptorDimensionality::serialize(std::ostream& out) const
{
  eigen_extensions::serialize(num_elements_, out);
}

void DescriptorDimensionality::deserialize(std::istream& in)
{
  eigen_extensions::deserialize(in, &num_elements_);
}

void DescriptorDimensionality::_applyNameTranslator(const std::string& id, const NameTranslator& translator)
{
  ROS_ASSERT(id == "dmap");
  translator.translate(&num_elements_, -1);
}

  
/************************************************************
 * Dataset
 ************************************************************/

void Dataset::serialize(std::ostream& out) const
{
  serializeNameMappings(out);
  
  int buf = instances_.size();
  out.write((char*)&buf, sizeof(buf));
  for(size_t i = 0; i < instances_.size(); ++i)
    out << instances_[i];
}

void Dataset::deserialize(std::istream& in)
{
  deserializeNameMappings(in);
  
  int buf;
  in.read((char*)&buf, sizeof(buf));
  instances_.resize(buf);
  for(size_t i = 0; i < instances_.size(); ++i)
    in >> instances_[i];
}

Dataset& Dataset::operator+=(const Dataset& other)
{
  ROS_ASSERT(nameMappingsAreEqual(other));

  // TODO: Maybe check that the inserted instances are of the right size.
  
  // Instances are copy-constructable.
  instances_.insert(instances_.end(), other.instances_.begin(), other.instances_.end());

  return *this;
}

bool Dataset::operator==(const Dataset& other) const
{
  if(!nameMappingsAreEqual(other))
    return false;
  if(instances_.size() != other.instances_.size())
    return false;
  for(size_t i = 0; i < instances_.size(); ++i)
    if(instances_[i] != other.instances_[i])
      return false;

  return true;
}

void Dataset::_applyNameTranslator(const std::string& id, const NameTranslator& translator)
{
  ROS_ASSERT(id == "cmap" || id == "dmap");
  for(size_t i = 0; i < instances_.size(); ++i)
    instances_[i].applyNameTranslator(id, translator);
}

size_t Dataset::numBytes() const
{
  size_t num = 0;
  for(size_t i = 0; i < instances_.size(); ++i)
    num += instances_[i].numBytes();

  return num;
}

void Dataset::deleteDescriptors()
{
  for(size_t i = 0; i < instances_.size(); ++i)
    instances_[i].deleteDescriptors();
}

std::string Dataset::status(const std::string& prefix, bool show_name_mappings) const
{
  ostringstream oss;
  if(show_name_mappings)
    oss << nameMappingStatus(prefix);
  
  oss << prefix << "Dataset memory usage: " << numBytes() << " bytes." << endl;
  oss << prefix << instances_.size() << " instances." << endl;
  oss << prefix << "Class count: " << endl;
  for(size_t i = 0; i < nameMapping("cmap").size(); ++i) {
    int num = 0;
    for(size_t j = 0; j < instances_.size(); ++j)
      if(instances_[j].label_[i] > 0)
	++num;
    oss << prefix << "  " << nameMapping("cmap").toName(i) << ": " << num << endl;
  }
  return oss.str();
}

void Dataset::setImportance(float importance)
{
  for(size_t i = 0; i < instances_.size(); ++i)
    instances_[i].label_ = instances_[i].label_.sign() * importance;
}

void Dataset::setLabel(const Label& label)
{
  ROS_ASSERT(label.rows() == (int)nameMapping("cmap").size());
  for(size_t i = 0; i < instances_.size(); ++i)
    instances_[i].label_ = label;
}

const Label& Dataset::label() const
{
  ROS_ASSERT(size() > 0);
  for(size_t j = 1; j < size(); ++j)
    ROS_ASSERT(instances_[j].label_ == instances_[j-1].label_);
  return instances_[0].label_;
}

Dataset::Ptr Dataset::clone() const
{
  return Dataset::Ptr(new Dataset(*this));
}

double Dataset::hash() const
{
  double val = 0;
  for(size_t i = 0; i < instances_.size(); ++i)
    val += instances_[i].hash();
  return val;
}

/************************************************************
 * TrackDataset
 ************************************************************/

void TrackDataset::setImportance(float importance)
{
  for(size_t i = 0; i < tracks_.size(); ++i)
    tracks_[i]->setImportance(importance);
}

DescriptorDimensionality TrackDataset::inferDescriptorDimensionality() const
{
  ROS_ASSERT(!tracks_.empty());
  ROS_ASSERT(!tracks_[0]->empty());

  // -- Determine DescriptorDimensionality.
  DescriptorDimensionality dim((*tracks_[0])[0].descriptors_.size());
  for(int i = 0; i < dim.num_elements_.rows(); ++i) {
    bool flag = false;
    for(size_t j = 0; !flag && j < size(); ++j) {
      for(size_t k = 0; !flag && k < tracks_[j]->size(); ++k) {
        const Instance& inst = (*tracks_[j])[k];
        if(inst[i]) {
          flag = true;
          dim.num_elements_(i) = inst[i]->rows();
        }
      }
    }
    // If no non-empty descriptors exist, this function will fail.
    ROS_ASSERT(flag);
  }

  return dim;
}

void TrackDataset::serialize(std::ostream& out) const
{
  out << "TrackDataset v2" << endl;
  serializeNameMappings(out);
  int buf = tracks_.size();
  out.write((char*)&buf, sizeof(buf));
  for(size_t i = 0; i < tracks_.size(); ++i)
    out << *tracks_[i];
}

void TrackDataset::deserialize(std::istream& in)
{
  string version;
  getline(in, version);
  ROS_ASSERT(version == "TrackDataset v2");
  
  deserializeNameMappings(in);
  int buf;
  in.read((char*)&buf, sizeof(buf));
  tracks_.resize(buf);
  for(size_t i = 0; i < tracks_.size(); ++i) {
    tracks_[i] = Dataset::Ptr(new Dataset);
    in >> *tracks_[i];
  }
}

void TrackDataset::_applyNameTranslator(const std::string& id, const NameTranslator& translator)
{
  for(size_t i = 0; i < tracks_.size(); ++i) {
    ROS_ASSERT(tracks_[i]);
    tracks_[i]->applyNameTranslator(id, translator);
  }
}

TrackDataset::Ptr TrackDataset::clone() const
{
  TrackDataset::Ptr td(new TrackDataset(*this));  // Shallow copy.  Includes name mapping, etc.
  // Deep copy of the tracks.
  for(size_t i = 0; i < td->size(); ++i)
    td->tracks_[i] = tracks_[i]->clone();

  return td;
}

bool TrackDataset::operator==(const TrackDataset& other) const
{
  if(!nameMappingsAreEqual(other))
    return false;
  
  if(tracks_.size() != other.tracks_.size())
    return false;
  
  for(size_t i = 0; i < tracks_.size(); ++i)
    if(*tracks_[i] != *other.tracks_[i])
      return false;

  return true;
}

TrackDataset& TrackDataset::operator+=(const TrackDataset& other) 
{
  // -- Check that the custom data is all the same.
  if(!tracks_.empty()) {
    
    const type_info& ti = (*tracks_[0])[0].raw_.type();
    for(size_t i = 0; i < tracks_.size(); ++i)
      for(size_t j = 0; j < tracks_[i]->size(); ++j)
        ROS_ASSERT((*tracks_[i])[j].raw_.type() == ti);
    
    if(!other.tracks_.empty())
      for(size_t i = 0; i < other.size(); ++i)
        for(size_t j = 0; j < other[i].size(); ++j)
          ROS_ASSERT(other[i][j].raw_.type() == ti);
  }

  // -- For now, don't deal with changing name mappings.
  //    Just make sure they're the same.

  // if(nameMappings().empty())
  //   applyNameMappings(other);
  ROS_ASSERT(nameMappingsAreEqual(other));

  // -- Add the new tracks.
  tracks_.insert(tracks_.end(), other.tracks_.begin(), other.tracks_.end());
  return *this;
}

size_t TrackDataset::numBytes() const
{
  size_t num = 0;
  for(size_t i = 0; i < tracks_.size(); ++i)
    num += tracks_[i]->numBytes();
  return num;
}

std::string TrackDataset::status(const std::string& prefix, bool show_name_mappings) const
{
  ostringstream oss;
  if(show_name_mappings)
    oss << nameMappingStatus(prefix);
  
  oss << prefix << "TrackDataset memory usage: " << numBytes() << " bytes." << endl;
  oss << prefix << tracks_.size() << " tracks." << endl;
  double total_importance = 0;
  for(size_t i = 0; i < tracks_.size(); ++i)
    for(size_t j = 0; j < track(i).size(); ++j)
      total_importance += track(i)[j].label_.array().abs().sum();
  if(!tracks_.empty()) {
    oss << prefix << "Average frame importance: " << total_importance / (totalInstances() * nameMapping("cmap").size()) << endl;
    oss << prefix << "Average track importance: " << total_importance / (tracks_.size() * nameMapping("cmap").size()) << endl;
    oss << prefix << "Average track length: " << totalInstances() / tracks_.size() << endl;
  }
  else {
    oss << prefix << "Average frame importance: " << 0 << endl;
    oss << prefix << "Average track importance: " << 0 << endl;
    oss << prefix << "Average track length: " << 0 << endl;
  }
  int num_empty = 0;
  for(size_t i = 0; i < tracks_.size(); ++i)
    if(track(i).size() == 0)
      ++num_empty;
  oss << prefix << num_empty << " empty tracks." << endl;

  for(size_t i = 0; i < nameMapping("cmap").size(); ++i) {
    int pos = 0;
    int neg = 0;
    int unk = 0;
    for(size_t j = 0; j < tracks_.size(); ++j) {
      if(track(j).size() == 0)
	continue;
      float y = label(j)(i);
      if(y == 0)
	++unk;
      else if(y > 0)
	++pos;
      else
	++neg;
    }
    oss << prefix << "  " << nameMapping("cmap").toName(i) << ": "
	<< pos << " positive, " << neg << " negative, " << unk << " unknown." << endl;
  }

  VectorXd buffer_count(nameMapping("cmap").size());
  buffer_count.setZero();
  for(size_t i = 0; i < tracks_.size(); ++i)
    if(track(i).size() > 0)
      for(int j = 0; j < buffer_count.rows(); ++j)
	if(label(i)(j) > 0)
	  ++buffer_count(j);
  oss << prefix << "Class proportions (tracks): " << buffer_count.transpose() / (double)tracks_.size() << endl;
  
  oss << prefix << totalInstances() << " frames." << endl;
  oss << prefix << "Mean frames / track: " << (double)totalInstances() / tracks_.size() << endl;
  for(size_t i = 0; i < nameMapping("cmap").size(); ++i) {
    int pos = 0;
    int neg = 0;
    int unk = 0;
    for(size_t j = 0; j < tracks_.size(); ++j) {
      for(size_t k = 0; k < track(j).size(); ++k) {
	float y = track(j)[k].label_(i);
	if(y == 0)
	  ++unk;
	else if(y > 0)
	  ++pos;
	else
	  ++neg;
      }
    }
    oss << prefix << "  " << nameMapping("cmap").toName(i) << ": "
	<< pos << " positive, " << neg << " negative, " << unk << " unknown." << endl;
  }

  return oss.str();
}

size_t TrackDataset::totalInstances() const
{
  size_t total = 0;
  for(size_t i = 0; i < tracks_.size(); ++i)
    total += tracks_[i]->size();
  return total;
}

const Label& TrackDataset::label(size_t i) const
{
  ROS_ASSERT(i < tracks_.size());
  ROS_ASSERT(tracks_[i]->size() > 0);
  
  for(size_t j = 1; j < tracks_[i]->size(); ++j)
    ROS_ASSERT(track(i)[j].label_ == track(i)[j-1].label_);

  return track(i)[0].label_;
}

std::vector<Label> TrackDataset::labels() const
{
  vector<Label> labels(size());
  for(size_t i = 0; i < size(); ++i)
    labels[i] = label(i);
  return labels;
}

void TrackDataset::load(const std::string& filename)
{
  ifstream f;
  f.open(filename.c_str());
  if(!f.is_open()) {
    cerr << "Failed to open " << filename << endl;
    assert(f.is_open());
  }
  deserialize(f);
  f.close();
}

TrackDataset::~TrackDataset()
{
}

/************************************************************
 * Helper functions
 ************************************************************/

NameMapping getStubDescriptorMap(int num_descriptors)
{
  NameMapping dmap;
  for(int i = 0; i < num_descriptors; ++i) { 
    ostringstream oss;
    oss << "dspace" << i;
    dmap.addName(oss.str());
  }
  return dmap;
}
  
float sign(float val)
{
  if(val > 0)
    return +1;
  else if(val < 0)
    return -1;
  else
    return 0;
}

void cropTracks(size_t max_length, TrackDataset* dataset)
{
  TrackDataset& d = *dataset;
  
  for(size_t i = 0; i < d.size(); ++i) {
    Dataset& track = d[i];
    if(track.size() <= max_length)
      continue;

    ROS_ASSERT(track.instances_.begin() + max_length < track.instances_.end());
    track.instances_.erase(track.instances_.begin() + max_length, track.instances_.end());

    ROS_ASSERT(track.size() == max_length);
  }

  ROS_ASSERT(d.totalInstances() <= max_length * d.size());
}


void splitTracks(size_t max_length, TrackDataset* dataset)
{
  TrackDataset& d = *dataset;
  size_t orig_total_instances = d.totalInstances();

  for(size_t i = 0; i < d.size(); ++i) {
    while(d[i].size() > max_length) {
      d.tracks_.push_back(Dataset::Ptr(new Dataset));
      Dataset& src = d[i];
      Dataset& dst = *d.tracks_.back();

      dst.applyNameMappings(src);
      
      vector<Instance>::iterator start = src.instances_.end() - max_length;
      vector<Instance>::iterator end = src.instances_.end();
      dst.instances_.insert(dst.instances_.begin(), start, end);
      src.instances_.erase(start, end);
      ROS_ASSERT(dst.size() == max_length);
    }
  }

  ROS_ASSERT(d.totalInstances() == orig_total_instances);      
}

void splitTracksFixedLength(size_t length, TrackDataset* dataset)
{
  TrackDataset& d = *dataset;
  vector<Dataset::Ptr> tracks;  // probably should reserve some amount.

  for(size_t i = 0; i < d.size(); ++i) {
    if(d[i].size() < length)
      continue;
    
    while(d[i].size() >= length) {
      tracks.push_back(Dataset::Ptr(new Dataset));
      Dataset& src = d[i];
      Dataset& dst = *tracks.back();

      dst.applyNameMappings(src);
      
      vector<Instance>::iterator start = src.instances_.end() - length;
      vector<Instance>::iterator end = src.instances_.end();
      dst.instances_.insert(dst.instances_.begin(), start, end);
      src.instances_.erase(start, end);
    }
  }

  d.tracks_ = tracks;
  for(size_t i = 0; i < d.tracks_.size(); ++i)
    ROS_ASSERT(d[i].size() == length);
}

TrackDataset::Ptr loadDatasets(const std::vector<std::string> paths,
                               const NameMapping& cmap,
                               bool verbose)
{
  ROS_ASSERT(!paths.empty());

  ROS_DEBUG_STREAM("loadDatasets called with Instance::custom_serializer_ of "
                   << Instance::custom_serializer_->name());
      
  TrackDataset::Ptr data(new TrackDataset());
  data->load(paths[0]);
  if(!cmap.empty())
    data->applyNameMapping("cmap", cmap);
  for(size_t i = 1; i < paths.size(); ++i) {
    if(verbose)
      cout << "Loading \"" << paths[i] << "\"" << endl;
    TrackDataset tmp;
    tmp.load(paths[i]);
    if(!cmap.empty())
      tmp.applyNameMapping("cmap", cmap);
    ROS_ASSERT(data->nameMappingsAreEqual(tmp));
    *data += tmp;
  }

  return data;
}

void saveByClassAndLabel(const TrackDataset& td, const std::string& basepath)
{
  for(size_t c = 0; c < td.nameMapping("cmap").size(); ++c) {
    TrackDataset part_pos;
    part_pos.applyNameMappings(td);
    part_pos.tracks_.reserve(td.size());
    for(size_t i = 0; i < td.size(); ++i) {
      const Label& label = td[i].label();
      if(label(c) > 0)
        part_pos.tracks_.push_back(td.tracks_[i]);
    }
    part_pos.save(basepath + "-" + td.nameMapping("cmap").toName(c) + "-pos.td");

    TrackDataset part_neg;
    part_neg.applyNameMappings(td);
    part_neg.tracks_.reserve(td.size());
    for(size_t i = 0; i < td.size(); ++i) {
      const Label& label = td[i].label();
      if(label(c) < 0)
        part_neg.tracks_.push_back(td.tracks_[i]);
    }
    part_neg.save(basepath + "-" + td.nameMapping("cmap").toName(c) + "-neg.td");
  }
}

void removeDuplicates(TrackDataset* td)
{
  vector<Dataset::Ptr> tracks;
  set<double> hashes;
  for(size_t i = 0; i < td->size(); ++i) {
    double hash = (*td)[i].hash();
    if(!hashes.count(hash)) {
      hashes.insert(hash);
      tracks.push_back(td->tracks_[i]);
    }
  }
  td->tracks_ = tracks;
}
