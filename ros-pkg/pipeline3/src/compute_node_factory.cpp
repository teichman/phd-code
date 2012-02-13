#include <pipeline/compute_node_factory.h>

using namespace std;

#define REGISTER_SERIALIZATION(T) if(dynamic_cast<T*>(node)) oss << #T << endl;
#define REGISTER_DESERIALIZATION(T) if(type.compare(#T) == 0) return new T(name, params);

namespace pipeline
{

  ComputeNodeFactory::ComputeNodeFactory()
  {
  }
  
  ComputeNodeFactory::~ComputeNodeFactory()
  {
  }

  // TODO: Factor this out so that the REGISTERs all appear in one method with little else around.
  std::string ComputeNodeFactory::serializeNode(ComputeNode* node) const
  {
    ostringstream oss;
    oss << node->getName() << endl;

    REGISTER_SERIALIZATION(EntryPoint<int>);
    REGISTER_SERIALIZATION(PlaceholderNode);
    
    map<string, Outlet*> inputs = node->getInputs();
    oss << "Inputs (" << inputs.size() << ")" << endl;
    map<string, Outlet*>::const_iterator it;
    for(it = inputs.begin(); it != inputs.end(); ++it) { 
      oss << it->first << " <- " << it->second->getNode()->getName()
	  << " " << it->second->getName() << endl;
    }
    oss << node->getParams() << endl;
    return oss.str();
  }

  ComputeNode* ComputeNodeFactory::deserializeNode(std::istream& in,
						   std::vector<std::string>* input_lines) const
  {
    string name;
    in >> name;
    string type;
    in >> type;

    string buf;
    in >> buf;
    ROS_ASSERT(buf.compare("Inputs") == 0);
    in >> buf;
    size_t num_inputs = atoi(buf.substr(1, buf.size() - 1).c_str());
    getline(in, buf);
    
    for(size_t i = 0; i < num_inputs; ++i) {
      getline(in, buf);
      input_lines->push_back(name + " " + buf);
    }
    
    Params params;
    params.deserialize(in);

    REGISTER_DESERIALIZATION(EntryPoint<int>);
    REGISTER_DESERIALIZATION(PlaceholderNode);

    return NULL;
  }
  
} // namespace pipeline
