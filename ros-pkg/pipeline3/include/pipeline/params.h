#ifndef PARAMS_H
#define PARAMS_H

#include <map>
#include <ros/console.h>
#include <ros/assert.h>
#include <boost/any.hpp>
#include <serializable/serializable.h>

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
    
    template<typename T> void set(const std::string& name, T val)
      {
	assertValid(name);
	storage_[name] = val;
      }
    
    template<typename T> T get(const std::string& name) const
      {
	assertValid(name);
	ROS_FATAL_STREAM_COND(storage_.count(name) == 0,
			      "Params does not have a member named \""
			      << name << "\"" << std::flush);
	ROS_ASSERT(storage_.count(name) == 1);

	try {
	  boost::any_cast<T>(storage_.find(name)->second);
	}
	catch(boost::bad_any_cast& e) {
	  ROS_FATAL_STREAM("Bad type when getting param \"" << name << "\"" << std::flush);
	  abort();
	}
	return boost::any_cast<T>(storage_.find(name)->second);
      }

  private:
    void assertValid(const std::string& name) const;
  };

}

#endif // PARAMS_H
