#include <online_learning/dataset.h>
#include <online_learning/instance_serializer.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <eigen_extensions/eigen_extensions.h>

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
