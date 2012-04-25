#ifndef DESCRIPTOR_DATABASE_H
#define DESCRIPTOR_DATABASE_H

#include <Eigen/Eigen>
#include <opencv2/core/core.hpp>
#include <boost/shared_ptr.hpp>
#include <serializable/serializable.h>
#include <xpl_calibration/orb_extractor.h>


/** \brief @b Abstract base class which contains a query-able
 * library of orb descriptors.
 */
class DescriptorDatabase
{
public:
  typedef Eigen::Matrix<uchar, 32, Eigen::Dynamic> PackedDescriptors;
  typedef boost::shared_ptr<PackedDescriptors> PackedDescriptorsPtr;
  typedef boost::shared_ptr<const PackedDescriptors> PackedDescriptorsConstPtr;
  typedef Eigen::Matrix<uchar, 32, 1> PackedDescriptor;
  typedef boost::shared_ptr<DescriptorDatabase> Ptr;

  //! descriptor length x num descriptors.
  PackedDescriptorsConstPtr packed_descriptors_;

  DescriptorDatabase(PackedDescriptorsConstPtr packed_descriptors);
  //! Fills indices (column numbers) with all descriptors within threshold of d.
  //! Returns the index of the closest, or -1 if none within threshold.
  virtual int query(const PackedDescriptor& d,
		    std::vector<int>* indices,
		    double threshold = 45) = 0;
  int size() const;

  double distSquared(const PackedDescriptor& a, const PackedDescriptor& b) const;
  friend class ProjectionIndex;
  
protected:
  std::vector<double> lookup_;
  
  double dot(const PackedDescriptor& a, const PackedDescriptor& b) const;
  double computeNumBitsSet(uchar val) const;
  void constructLookupTable();
};

class NaiveDescriptorDatabase : public DescriptorDatabase
{
public:
  typedef boost::shared_ptr<NaiveDescriptorDatabase> Ptr;
  
  NaiveDescriptorDatabase(PackedDescriptorsConstPtr packed_descriptors);
  //! Returns all descriptors within threshold of d.
  int query(const PackedDescriptor& d,
	    std::vector<int>* indices,
	    double thresh = 45);
};


class ProjectionIndex
{
public:
  typedef boost::shared_ptr<ProjectionIndex> Ptr;
  
  DescriptorDatabase::PackedDescriptorsConstPtr packed_descriptors_;
  int byte_number_;
  
  ProjectionIndex(DescriptorDatabase::PackedDescriptorsConstPtr packed_descriptors, int byte_number);
  const std::vector<int>& query(const DescriptorDatabase::PackedDescriptor& d) const;

private:
  std::vector< std::vector<int> > buckets_;
};

class LSHDescriptorDatabase : public DescriptorDatabase
{
public:
  LSHDescriptorDatabase(DescriptorDatabase::PackedDescriptorsConstPtr packed_descriptors);
  int query(const DescriptorDatabase::PackedDescriptor& d, std::vector<int>* indices, double thresh = 45);
  
private:
  std::vector<ProjectionIndex::Ptr> projection_indices_;
  //! flags_[i] == true if we've computed the distance from the query descriptor
  //! to database descriptor i.
  std::vector<bool> flags_;
};


std::string printBitString(const DescriptorDatabase::PackedDescriptor& descr);
  
#endif // DESCRIPTOR_DATABASE_H
