#include <xpl_calibration/descriptor_database.h>

using namespace std;


/************************************************************
 * DescriptorDatabase
 ************************************************************/

DescriptorDatabase::DescriptorDatabase(PackedDescriptorsConstPtr packed_descriptors) :
  packed_descriptors_(packed_descriptors)
{
  assert(packed_descriptors_);
  constructLookupTable();
}

int DescriptorDatabase::size() const
{
  return packed_descriptors_->cols();
}

void DescriptorDatabase::constructLookupTable()
{
  assert(sizeof(uchar) == 1);
  lookup_ = vector<double>(256);
  for(int i = 0; i < 256; ++i) { 
    lookup_[i] = computeNumBitsSet((uchar)i);
//    cout << std::hex << i << ": " << std::dec << lookup_[i] << endl;
  }
}

double DescriptorDatabase::computeNumBitsSet(uchar val) const
{
  double result = 0;
  uchar mask = 1;
  for(int i = 0; i < 8; ++i) {
    if(val & mask)
      ++result;
    mask = mask << 1;
  }

  return result;
}

double DescriptorDatabase::distSquared(const PackedDescriptor& a, const PackedDescriptor& b) const
{
  double result = 0;
  for(int i = 0; i < a.rows(); ++i) 
    result += lookup_[a.coeff(i, 0) ^ b.coeff(i, 0)];

  return result;
}


double DescriptorDatabase::dot(const PackedDescriptor& a, const PackedDescriptor& b) const
{
  double result = 0;
  for(int i = 0; i < a.rows(); ++i)
    result += lookup_[a.coeff(i, 0) & b.coeff(i, 0)];

  return result;
}


/************************************************************
 * NaiveDescriptorDatabase
 ************************************************************/

NaiveDescriptorDatabase::NaiveDescriptorDatabase(PackedDescriptorsConstPtr packed_descriptors) :
  DescriptorDatabase(packed_descriptors)
{

}

int NaiveDescriptorDatabase::query(const PackedDescriptor& d, std::vector<int>* indices, double threshold)
{
  assert(indices->capacity() == (size_t)packed_descriptors_->cols());
  assert(indices->empty());

  int best = -1;
  double min = FLT_MAX;
  for(int i = 0; i < packed_descriptors_->cols(); ++i) {
    double d2 = distSquared(packed_descriptors_->col(i), d);
    if(d2 < threshold) { 
      indices->push_back(i);
      if(d2 < min) { 
	min = d2;
	best = i;
      }
    }
  }
  
  return best;
}

/************************************************************
 * ProjectionIndex
 ************************************************************/

ProjectionIndex::ProjectionIndex(DescriptorDatabase::PackedDescriptorsConstPtr packed_descriptors,
				 int byte_number) :
  packed_descriptors_(packed_descriptors),
  byte_number_(byte_number),
  buckets_(256)
{
  // -- Fill the buckets.
  for(int i = 0; i < packed_descriptors_->cols(); ++i) {
    buckets_[packed_descriptors_->coeffRef(byte_number_, i)].push_back(i);
  }
}

const std::vector<int>& ProjectionIndex::query(const DescriptorDatabase::PackedDescriptor& d) const
{
  return buckets_[d(byte_number_, 0)];
}
  

/************************************************************
 * LSHDescriptorDatabase
 ************************************************************/

LSHDescriptorDatabase::LSHDescriptorDatabase(DescriptorDatabase::PackedDescriptorsConstPtr packed_descriptors) :
  DescriptorDatabase(packed_descriptors),
  flags_(packed_descriptors->cols(), false)
{
  int num_projections = 7;
  
  for(int i = 0; i < num_projections; ++i) {
    ProjectionIndex::Ptr pi(new ProjectionIndex(packed_descriptors, i*4));
    projection_indices_.push_back(pi);
  }
}

int LSHDescriptorDatabase::query(const DescriptorDatabase::PackedDescriptor& d, std::vector<int>* indices, double threshold)
{
  // -- Reset the flags.
  for(size_t i = 0; i < flags_.size(); ++i)
    flags_[i] = false;

  // -- Get all buckets and do linear search on their contents.
  int best = -1;
  double min_distance = FLT_MAX;
  for(size_t i = 0; i < projection_indices_.size(); ++i) {
    const vector<int>& bucket = projection_indices_[i]->query(d);
    for(size_t j = 0; j < bucket.size(); ++j) {
      int idx = bucket[j];
      if(flags_[idx])
	continue;

      flags_[idx] = true;
      double distance = distSquared(packed_descriptors_->col(idx), d);
      if(distance < threshold) { 
	indices->push_back(idx);

	if(distance < min_distance) {
	  min_distance = distance;
	  best = idx;
	}
      }
    }
  }

  return best;
}

