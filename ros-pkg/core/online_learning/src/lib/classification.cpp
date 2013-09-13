#include <online_learning/online_learning.h>

using namespace Eigen;
using namespace std;

namespace odontomachus
{

  Classification::Classification()
  {
  }
     
  Classification::Classification(const Eigen::VectorXf& response) :
    response_(response)
  {
  }

  void Classification::serialize(std::ostream& out) const
  {
    out << "Classification" << endl;
    out << response_.transpose() << endl;
    out << "classification_id: " << getClassId() << endl;
  }

  void Classification::deserialize(std::istream& in)
  {
    ROS_FATAL("No.");
    abort();
  }
  
  int Classification::getClassId() const
  {
    int id;
    float max = response_.maxCoeff(&id);
    if(max <= 0)
      id = -1;
    return id;
  }

  float Classification::getConfidence() const
  {
    return fabs(response_.maxCoeff());
  }

  std::string Classification::status(const NameMapping& class_map) const
  {
    ostringstream oss;
    if(response_.rows() == 0) {
      oss << "   (Uninitialized)" << endl;
    }
    else {
      int id = getClassId();
      for(int i = 0; i < response_.rows(); ++i) {
	if(id == i)
	  oss << "** ";
	else
	  oss << "   ";
	oss << class_map.toName(i) << ": " << response_(i) << endl;
      }
    }

    return oss.str();
  }
  
} // namespace
