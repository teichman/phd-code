#include <pipeline/params.h>

using namespace std;
using boost::any;
using boost::any_cast;

namespace pipeline
{
  
  void Params::serialize(std::ostream& out) const
  {
    out << "Params (" << storage_.size() << ")" << endl;
    map<string, any>::const_iterator it;
    for(it = storage_.begin(); it != storage_.end(); ++it) {
      string name = it->first;
      any a = it->second;
      bool success = false;
      try {
	any_cast<string>(a);
	out << "string " << name << " " << any_cast<string>(a) << endl;
	success = true;
      }
      catch(boost::bad_any_cast& e) {}
      try {
	any_cast<int>(a);
	out << "int " << name << " " << any_cast<int>(a) << endl;
	success = true;
      }
      catch(boost::bad_any_cast& e) {}
      try {
	any_cast<double>(a);
	out << "double " << name << " " << any_cast<double>(a) << endl;
	success = true;
      }
      catch(boost::bad_any_cast& e) {}

      ROS_FATAL_STREAM_COND(!success, "Params object contains data that it does not know how to serialize." << flush);
      ROS_ASSERT(success);
    }
  }

  void Params::deserialize(std::istream& in)
  {
    string buf;
    in >> buf;
    ROS_ASSERT(buf.compare("Params") == 0);
    in >> buf;
    size_t num_params = atoi(buf.substr(1, buf.size() - 1).c_str());
    getline(in, buf);

    for(size_t i = 0; i < num_params; ++i) {
      string type;
      string name;
      in >> type;
      in >> name;
      
      if(type.compare("string") == 0) {
	string val;
	in >> val;
	set<string>(name, val);
      }
      else if(type.compare("int") == 0) {
	int val;
	in >> val;
	set<int>(name, val);
      }
      else if(type.compare("double") == 0) {
	double val;
	in >> val;
	set<double>(name, val);
      }
      else {
	ROS_FATAL_STREAM("Deserialization failed: type \"" << type << "\" not recognized." << flush);
	abort();
      }
      getline(in, buf);
    }
            
  }

  void Params::assertValid(const std::string& name) const
  {
    ROS_ASSERT(name.find(" ") == std::string::npos);
  }

} // namespace pipeline
