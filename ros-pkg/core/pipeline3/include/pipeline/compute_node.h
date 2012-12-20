#ifndef COMPUTE_NODE_H
#define COMPUTE_NODE_H

#include <iostream>
#include <map>
#include <locale>
#include <list>
#include <sys/time.h>
#include <set>
#include <queue>
#include <stdint.h>
#include <fstream>
#include <pthread.h>
#include <errno.h>
#include <boost/shared_ptr.hpp>
#include <ros/console.h>
#include <ros/assert.h>
#include <bag_of_tricks/high_res_timer.h>
#include <pipeline/params.h>

namespace pipeline
{
  class ComputeNode;

  //! Generic class for passing data out of a pipeline::ComputeNode.
  //! T will be copied a bit so it should generally be something small,
  //! like a pointer, shared_ptr, or POD type.
  //!
  //! Convention: nodes that pass out shared_ptrs will reallocate every time
  //! rather than erase the data from under the pointer.  Note that reallocation
  //! is generally expensive and that you should probably be using fixed storage
  //! whenever possible, and in this case your Outlets should pass out
  //! pointers.
  //!
  //! Finally, you should probably prefer const pointers or const shared_ptrs.
  //! Different threads executing downstream nodes are likely to simultaneously read the data
  //! that these pointers point to, so nothing should be modifying them.
  class Outlet
  {
  public:
    bool has_data_;
    
    //! T is set with the default constructor, so you should probably set it with push() after
    //! construction.
    Outlet(std::string name, ComputeNode* node);
    //! Only the descendent nodes should pull.
    template <typename T> T pull() const;
    //! Only the owner should push.
    //! node_->flush() should call this with something appropriate, like NULL if T is a pointer.
    template<typename T> void push(T data) { data_ = data; has_data_ = true; }
    void flush() { data_ = (void*)NULL; has_data_ = false; }
    ComputeNode* getNode() const { return node_; }
    std::string getName() const { return name_; }
    
  protected:
    std::string name_;
    ComputeNode* node_;
    boost::any data_;
  };
  
  //! Abstract base class that represents a node in the computation graph.  All nodes inherit from this class, at least indirectly.
  //! In general, _foo() is a member function that is wrapped by foo().
  class ComputeNode {  
  public:
    //! Whether to pause after computation and display debugging output by running display()
    bool debug_;
    //! To compute, or not to compute?
    bool disabled_;
  
    ComputeNode(std::string name, Params params = Params());
    virtual ~ComputeNode();
    std::string getName() const { return name_; }
    Params getParams() const { return params_; }
    template<typename T> T param(const std::string& name) const;
    template<typename T> void declareParam(std::string name);
    void declareInput(std::string name);
    void declareOutput(std::string name);
    void registerInput(std::string name, Outlet* outlet);
    Outlet* getOutlet(std::string name) const;
    template<typename T> void push(std::string name, T val);
    template<typename T> T pull(std::string name) const;
    template<typename T> void pull(std::string name, T* dst) const;
    
    //! Returns computation time in ms.
    double getComputationTime() const;
    int getNumTimesComputed() const;
    //! Returns node name with the number of times it has been run.
    std::string runName(int width = 4) const;
    std::map<std::string, Outlet*> getInputs() const { return inputs_; }
    std::string printOutputs() const;
          
  protected:
    //! Performs computation, given data from nodes in upstream_.
    //! This is the primary method that must be overloaded by a class that inherits from ComputeNode.
    virtual void _compute() = 0;
    //! Display function, called on compute() when debug_ == true.
    virtual void _display() const;
    //! Hard reset; clears everything.
    virtual void _reset();
    
  private:
    std::string name_;
    Params params_;
    std::set<std::string> declared_inputs_;
    std::set<std::string> declared_params_;
    std::map<std::string, Outlet*> inputs_;
    std::map<std::string, Outlet*> outputs_;
    std::vector<ComputeNode*> upstream_;
    std::vector<ComputeNode*> downstream_;
    int num_finished_upstream_;
    bool done_computation_;
    bool started_computation_;
    bool on_queue_;
    pthread_mutex_t mutex_;
    //! The time it took to compute, in milliseconds.
    double time_msec_;
    int num_times_computed_;

    bool doneComputation() const {return done_computation_;}
    void assertValid(std::string name) const;
    //! Function called after node computes if debug_ == true.
    //! Virtual for overloading by ComputeNode subclasses that are abstract, e.g. DescriptorNode.
    //! This might not be necessary anymore.
    virtual void display() const; 
    void flush();
    void reset();
    void compute();
    bool ready();
    bool trylock();
    void lock();
    void unlock();
  
    friend class Pipeline;
    friend void* propagateComputation(void *pipeline);
    friend std::vector<ComputeNode*> getComponent(ComputeNode* node);
  };


  /************************************************************
   * Template definitions
   ************************************************************/
  
  template<typename T> T ComputeNode::param(const std::string& name) const
  {
    // Force the writer of a node to declare all his params.
    if(declared_params_.count(name) != 1) { 
      ROS_FATAL_STREAM("Param \"" << name << "\" was not declared.");
      abort();
    }
    return params_.get<T>(name);
  }
  
  template<typename T> void ComputeNode::declareParam(std::string name)
  {
    declared_params_.insert(name);
    params_.get<T>(name); // This will abort if the name doesn't exist or the type is wrong.
  }

  template<typename T> void ComputeNode::push(std::string name, T val)
  {
    if(outputs_.count(name) != 1) {
      ROS_FATAL_STREAM("Outlet named \"" << name << "\" was pushed to, but does not exist in this node.");
      ROS_FATAL_STREAM("Valid outputs in this node: " << printOutputs() << std::flush);
      ROS_ASSERT(outputs_.count(name) == 1);
    }
    
    outputs_[name]->push<T>(val);
  }

  template<typename T> T ComputeNode::pull(std::string name) const
  {
    ROS_ASSERT(inputs_.count(name) == 1);
    return inputs_.find(name)->second->pull<T>();
  }

  template<typename T> void ComputeNode::pull(std::string name, T* dst) const
  {
    ROS_ASSERT(inputs_.count(name) == 1);
    *dst = inputs_.find(name)->second->pull<T>();
  }

  template<typename T> T Outlet::pull() const
  {
    ROS_ASSERT(has_data_);
    try { 
      return boost::any_cast<T>(data_);
    }
    catch(boost::bad_any_cast& e) {
      // TODO: Add Outlet name, etc., here.
      ROS_FATAL_STREAM("Type mismatch during pull() from " << node_->getName() << "." << std::flush);
      abort();
    }
    return boost::any_cast<T>(data_);
  }
  
}

#endif // COMPUTE_NODE_H
