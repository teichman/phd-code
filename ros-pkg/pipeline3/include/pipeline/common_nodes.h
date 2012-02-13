#ifndef COMMON_NODES_H
#define COMMON_NODES_H

#include <pipeline/compute_node.h>

namespace pipeline
{

  //! Class to manage input of data to a Pipeline.
  //! T will be copied a bit so it should generally be something small,
  //! like a pointer, shared_ptr, or POD type.
  template<typename T>
  class EntryPoint : public ComputeNode
  {
  public:
    EntryPoint(std::string name, Params params = Params()) :
      ComputeNode(name, params)
    {
      declareOutput("outlet");
    }
    
    void setData(T data) { push<T>("outlet", data); }
    void _compute() {}
  };
  
  //! Temporary, for testing.
  class PlaceholderNode : public ComputeNode
  {
  public:
    PlaceholderNode(std::string name, Params params) :
      ComputeNode(name, params)
    {
      declareParam<int>("foo");
      declareInput("incoming");
      declareOutput("outgoing");
    }

    void _compute();
  };

}

#endif // COMMON_NODES_H
