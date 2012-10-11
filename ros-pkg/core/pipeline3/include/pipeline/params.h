#ifndef PARAMS_H
#define PARAMS_H

#include <map>
#include <boost/any.hpp>
#include <serializable/serializable.h>
#include <pipeline/common.h>

namespace pipeline
{
  
  class Params : public Serializable
  {
  public:
    std::map<std::string, boost::any> storage_;

    Params() {}
    ~Params() {}
    void serialize(std::ostream& out) const;
    void deserialize(std::istream& in);
    bool exists(const std::string& name) const;
    bool operator==(const Params& other) const;
    bool operator<(const Params& other) const;
    
    template<typename T> bool isType(const std::string& name) const
    {
      if(storage_.count(name) == 0) { 
	PL_ABORT("Tried to check type of non-existent param \"" << name << "\".");
      }
            
      bool is_T = true;
      try {
	boost::any_cast<T>(storage_.find(name)->second);
      }
      catch(boost::bad_any_cast& e) {
	is_T = false;
      }

      return is_T;
    }
    
    template<typename T> void set(const std::string& name, const T& val)
    {
      assertValidName(name);
      if(storage_.count(name) != 0 && !isType<T>(name))
	PL_ABORT("Tried to change type of param \"" << name << "\".");
      
      storage_[name] = val;
    }
    
    template<typename T> T get(const std::string& name) const
    {
      assertValidName(name);
      if(storage_.count(name) == 0) { 
	PL_ABORT("Params does not have a member named \"" << name << "\"");
      }

      try {
	boost::any_cast<T>(storage_.find(name)->second);
      }
      catch(boost::bad_any_cast& e) {
	PL_ABORT("Bad type when getting param \"" << name << "\"");
      }
      return boost::any_cast<T>(storage_.find(name)->second);
    }

    
  };

}

#endif // PARAMS_H
