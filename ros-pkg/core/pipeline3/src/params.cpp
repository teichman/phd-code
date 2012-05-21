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
      try {
	any_cast<bool>(a);
	out << "bool " << name << " " << any_cast<bool>(a) << endl;
	success = true;
      }
      catch(boost::bad_any_cast& e) {}

      if(!success) { 
	PL_ABORT("Tried to serialize Param field \"" << name << "\", but Params does not know how to serialize its data type.");
      }
    }
  }

  void Params::deserialize(std::istream& in)
  {
    string buf;
    in >> buf;
    PL_ASSERT(buf.compare("Params") == 0);
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
      else if(type.compare("bool") == 0) {
	bool val;
	in >> val;
	set<bool>(name, val);
      }
      else {
	PL_ABORT("Deserialization failed: type \"" << type << "\" not recognized.");
      }
      getline(in, buf);
    }
            
  }

  bool Params::exists(const std::string& name) const
  {
    return (storage_.count(name) != 0);
  }
  
} // namespace pipeline
