#ifndef PIPELINE_H
#define PIPELINE_H

#include <pipeline/compute_node_factory.h>

namespace pipeline
{

  //! Class that represents the entire computation graph and manages its execution.
  class Pipeline : public Serializable {
  public:
    std::vector<ComputeNode*> nodes_;
    
    Pipeline(int num_threads);
    ~Pipeline();
    void serialize(std::ostream& out) const;
    void deserialize(std::istream& in);
    
    //! Return all nodes of type T for which the given the test function evaluates to true.
    template<typename T> std::vector<T*> filterNodes(bool (*test)(T* node) = NULL) const;
    //! There must be only one node of type T that passes the test.
    template<typename T> T* getNode() const;
    
    //! Adds all nodes that are connected to node.
    void addComponent(ComputeNode* node);
    //! Sets the debug_ flag on all nodes.
    //! If debug is being turned on, it kills the thread pool and restarts it with only one thread.
    //! If debug is being turned off, it kills the thread pool and restarts it with num_threads_.
    void setDebug(bool debug);
    void setNumThreads(int num_threads);
    bool getDebug() const;
    
    //! Clears all cached data in the pipeline, i.e. calls flush on all nodes.
    void flush();
    //! Calls _reset() on all nodes.
    void reset();
    //! Runs all computation until completion.
    void compute();
    //! Returns a string with timing results for each node name.  Nodes that have identical getShortName()s will be averaged together.
    std::string reportTiming();
    std::string getGraphviz() const;
    void writeGraphviz(const std::string& filename) const;
    //! Disables node and all other nodes connected to it.
    int disableComponent(ComputeNode* node);
    //! Enables node and all other nodes connected to it.
    int enableComponent(ComputeNode* node);
    void enableAll();
    void disableAll();
    //! You can use this to make serialization and deserialization work for
    //! your own ComputeNode subclasses.  Will be deleted by Pipeline on destruction.
    void setFactory(ComputeNodeFactory* cnf);
    
  protected:
    //! Makes sure that all nodes in the pipeline are actually in nodes_.
    //! This prevents the possibility of a very nasty bug.
    void assertCompleteness();
  
  private:
    std::vector<pthread_t> threads_;
    //! Not necessarily equal to threads_.size(); see setDebug().
    int num_threads_;
    bool debug_;
    pthread_mutex_t mutex_;
    pthread_cond_t queue_cv_;
    pthread_cond_t done_cv_;

    std::vector<ComputeNode*> queue_;
    bool done_computation_;
    size_t num_nodes_computing_;
    bool destructing_;
    //! The amount of time it took from calling compute() to being done.
    double time_msec_;
    std::set<std::string> node_names_;
    ComputeNodeFactory* cnf_;
    
    //! No copy constructing of Pipelines.
    Pipeline(const Pipeline& other);
    //! No assigment of Pipelines.
    Pipeline& operator=(const Pipeline& p);
    ComputeNode* getReadyNode();
    void assertNoDuplicates() const;
    void registerCompleted(ComputeNode* node);
    bool trylock();
    void lock();
    void unlock();
    //! Wait on the queue condition variable.
    void wait();
    //! Signal the queue condition variable.
    void signal();
    void spawnThreadPool(int num_threads);
    void killThreadPool();
    bool computing();
    int switchComponent(ComputeNode* node, bool disabled);
    void run();
    
    friend void* propagateComputation(void *pipeline);
  };


  /****************************************
   * Helper Functions
   ****************************************/

  std::vector<ComputeNode*> getComponent(ComputeNode* node);
  
  //! Returns nodes of type T that pass a user-specified test.
  template<typename T>
  std::vector<T*>
  filterNodes(const std::vector<ComputeNode*>& nodes, bool (*test)(T* node) = NULL);

  //! Function that Pipeline worker threads call.
  void* propagateComputation(void *pipeline);

 
  /*****************************************
   * Function Templates
   ****************************************/
  
  template<typename T>
  std::vector<T*> Pipeline::filterNodes(bool (*test)(T* node)) const
  {
    return Pipeline::filterNodes<T>(nodes_, test); // Pipeline:: is required; otherwise g++ thinks that we're calling a member function which doesn't exist.
  }
  
  template<typename T>
  std::vector<T*> filterNodes(const std::vector<ComputeNode*>& nodes, bool (*test)(T* node))
  {
    std::vector<T*> passed;
    passed.reserve(nodes.size());
    for(size_t i = 0; i < nodes.size(); ++i) {
      T* casted = dynamic_cast<T*>(nodes[i]);
      if(!casted)
	continue;
      if(test && !test(casted))
	continue;
    
      passed.push_back(casted);
    }
    
    return passed;
  }

  //! Returns the one node of type T.  There must be only one.
  template<typename T> T* getNode(const std::vector<ComputeNode*>& nodes);

  template<typename T>
  T* Pipeline::getNode() const
  {
    return pipeline::getNode<T>(nodes_);
  }
  
  template<typename T>
  T* getNode(const std::vector<ComputeNode*>& nodes)
  {
    std::vector<T*> passed;
    passed.reserve(nodes.size());
    for(size_t i = 0; i < nodes.size(); ++i) {
      T* casted = dynamic_cast<T*>(nodes[i]);
      if(!casted)
	continue;
      passed.push_back(casted);
    }

    ROS_ASSERT(passed.size() == 1 || passed.size() == 0);
    if(passed.size() == 1)
      return passed[0];
    else
      return NULL;
  }


} // namespace pipeline

#endif // PIPELINE_H
