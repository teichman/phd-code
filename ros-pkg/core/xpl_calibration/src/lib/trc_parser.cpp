#include <xpl_calibration/trc_parser.h>

using namespace std;
using namespace rgbd;

TRCParser::TRCParser()
{
}

ostream& operator<<(ostream& out, const std::vector<double>& vec)
{
  for(size_t i = 0; i < vec.size(); ++i)
    out << vec[i] << " ";
  return out;
}
  
void TRCParser::deserialize(std::istream& in)
{
  string line;

  // Header
  getline(in, line);
  getline(in, line);
  getline(in, line);
  getline(in, line);
  getline(in, line);

  vector<double> fields;
  while(getline(in, line), !in.eof()) {
    explode(line, &fields);
    // cout << "\\\\\\\\\\\\\\\\\\\\" << endl;
    // cout << "line: " << line << endl;
    // cout << "fields: " << fields << endl;
    // cout << "num_fields" << fields.size() << endl;
    // cin.ignore();

    ROS_ASSERT((size_t)fields[0] == frames_.size() + 1);
    Cloud::Ptr frame(new Cloud);
    frame->header.stamp = (fields[1]) * 1e9;
    for(size_t i = 2; i < fields.size(); i+=3) {
      Point pt;
      pt.x = fields[i] * 0.001f;
      pt.y = fields[i+1] * 0.001f;
      pt.z = fields[i+2] * 0.001f;
      pt.r = 255;
      pt.g = 0;
      pt.b = 0;
      frame->push_back(pt);
    }
    frames_.push_back(frame);
    //cout << "Added frame " << fields[0] << " at ts = " << frame->header.stamp * 1e-9  << " with " << frame->size() << " points." << endl;
  }
}

void TRCParser::serialize(std::ostream& out) const
{
  assert(0);
}

rgbd::Cloud::Ptr TRCParser::getFrame(double ts, double tol) const
{
  return Cloud::Ptr();
}

void TRCParser::explode(const std::string& line, std::vector<double>* fields) const
{
  fields->clear();
  istringstream iss(line);
  double val;
  while(iss >> val, !iss.eof())
    fields->push_back(val);
}
  
