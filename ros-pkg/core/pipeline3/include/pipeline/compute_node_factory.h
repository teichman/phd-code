#ifndef COMPUTE_NODE_FACTORY_H
#define COMPUTE_NODE_FACTORY_H

#include <pipeline/compute_node.h>
#include <pipeline/common_nodes.h>

namespace pipeline
{

  //! Extend this class to make serializeNode and deserializeNode support
  //! your own ComputeNode subclasses.
  class ComputeNodeFactory
  {
  public:
    ComputeNodeFactory();
    virtual ~ComputeNodeFactory();
    virtual std::string serializeNode(ComputeNode* node) const;
    virtual ComputeNode* deserializeNode(std::istream& in, std::vector<std::string>* input_lines) const;
  };
  
}

#endif // COMPUTE_NODE_FACTORY_H
