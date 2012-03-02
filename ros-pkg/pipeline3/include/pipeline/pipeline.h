#ifndef PIPELINE_H
#define PIPELINE_H

#include <pipeline/pod.h>
#include <pipeline/common_pods.h>

namespace pipeline
{

  //! Class that represents the entire computation graph and manages its execution.
  class Pipeline : public Serializable {
  public:
    // ----------------------------------------
    // -- Setup
    // ----------------------------------------
    Pipeline(int num_threads);
    //! Adds all Pods that are connected to pod.
    void addConnectedComponent(Pod* pod);
    void addPods(const std::vector<Pod*> pods);

    // ----------------------------------------
    // -- Execution
    // ----------------------------------------
    //! Calls setData on an EntryPoint<T> with name pod_name.
    template<typename T> void setInput(const std::string& pod_name, T data);
    //! Runs all computation until completion.
    void compute();
    //! Calls reset() on all pods.
    void reset();
    //! Convenient way of pulling output from the Pipeline.
    template<typename T> T getOutput(const std::string& pod_name, const std::string& output_name) const;
    //! Convenient way of pulling output from the Pipeline.
    template<typename T> void getOutput(const std::string& pod_name,
					const std::string& output_name,
					T* dest) const;

    // ----------------------------------------
    // -- Control
    // ----------------------------------------
    //! Sets the debug_ flag on all pods.
    void setDebug(bool debug);
    bool getDebug() const;
    void setNumThreads(int num_threads);
    //! Disables pod and all other pods connected to it.
    int disableComponent(Pod* pod);
    //! Enables pod and all other pods connected to it.
    int enableComponent(Pod* pod);
    void enableAll();
    void disableAll();
    
    // ----------------------------------------
    // -- Pod access
    // ----------------------------------------
    //! Returns the Pod with the supplied name.
    Pod* getPod(const std::string& name) const;
    //! Returns the Pod of type T.  There must be exactly one.
    template<typename T> T* getPod() const;
    //! Returns the Pod of type T with the supplied name.
    template<typename T> T* getPod(const std::string& name) const;
    //! Return all pods of type T for which the given the test function evaluates to true.
    template<typename T> std::vector<T*> filterPods(bool (*test)(T* pod) = NULL) const;

    // ----------------------------------------
    // -- Reporting
    // ----------------------------------------
    std::string reportTiming() const;
    std::string reportSlowestPath() const;
    std::string getGraphviz() const;
    void writeGraphviz(const std::string& filename) const;
    
    // ----------------------------------------
    // -- Things you don't need to care about
    // ----------------------------------------
    ~Pipeline();
    void serialize(std::ostream& out) const;
    void deserialize(std::istream& in);
      
  private:
    std::vector<Pod*> pods_;
    std::vector<pthread_t> threads_;
    //! Not necessarily equal to threads_.size(); see setDebug().
    int num_threads_;
    bool debug_;
    pthread_mutex_t mutex_;
    pthread_cond_t queue_cv_;
    pthread_cond_t done_cv_;

    std::vector<Pod*> queue_;
    bool done_computation_;
    size_t num_pods_computing_;
    bool destructing_;
    //! The amount of time it took from calling compute() to being done.
    double time_msec_;
    std::map<std::string, Pod*> pod_names_;
    
    //! No copy constructing of Pipelines.
    Pipeline(const Pipeline& other);
    //! No assigment of Pipelines.
    Pipeline& operator=(const Pipeline& p);
    Pod* getReadyPod();
    void registerCompleted(Pod* pod);
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
    int switchComponent(Pod* pod, bool disabled);
    void run();
    //! Makes sure that all Pods in the Pipeline are actually in pods_.
    void assertCompleteness();
    //! Makes sure there are no duplicate Pods or duplicate Pod names in the Pipeline.
    void assertNoDuplicates() const;
    
    friend void* propagateComputation(void *pipeline);
  };


  /****************************************
   * Helper Functions
   ****************************************/

  std::vector<Pod*> getComponent(Pod* pod);
  
  //! Returns pods of type T that pass a user-specified test.
  template<typename T>
  std::vector<T*>
  filterPods(const std::vector<Pod*>& pods, bool (*test)(T* pod) = NULL);

  //! Function that Pipeline worker threads call.
  void* propagateComputation(void *pipeline);

 
  /*****************************************
   * Function Templates
   ****************************************/
  
  template<typename T>
  std::vector<T*> Pipeline::filterPods(bool (*test)(T* pod)) const
  {
    return Pipeline::filterPods<T>(pods_, test); // Pipeline:: is required; otherwise g++ thinks that we're calling a member function which doesn't exist.
  }
  
  template<typename T>
  std::vector<T*> filterPods(const std::vector<Pod*>& pods, bool (*test)(T* pod))
  {
    std::vector<T*> passed;
    passed.reserve(pods.size());
    for(size_t i = 0; i < pods.size(); ++i) {
      T* casted = dynamic_cast<T*>(pods[i]);
      if(!casted)
	continue;
      if(test && !test(casted))
	continue;
    
      passed.push_back(casted);
    }
    
    return passed;
  }
  
  //! Returns the one pod of type T.  There must be only one.
  template<typename T>
  T* getPod(const std::vector<Pod*>& pods)
  {
    std::vector<T*> passed;
    passed.reserve(pods.size());
    for(size_t i = 0; i < pods.size(); ++i) {
      T* casted = dynamic_cast<T*>(pods[i]);
      if(!casted)
	continue;
      passed.push_back(casted);
    }

    if(!(passed.size() == 1 || passed.size() == 0)) {
      PL_ABORT("Called getPod<T>(), but multiple Pods of type T were found."
	       << " You probably need to use getPod<T>(name).");
    }
    if(passed.size() == 1)
      return passed[0];
    else {
      PL_ABORT("Called getPod<T>(), but no Pods of type T were found.");
    }
    return NULL;
  }

  template<typename T>
  T* Pipeline::getPod() const
  {
    return pipeline::getPod<T>(pods_);
  }

  template<typename T> T* getPod(const std::string& name,
				  const std::vector<Pod*>& pods)
  {
    std::vector<T*> passed;
    for(size_t i = 0; i < pods.size(); ++i) {
      T* casted = dynamic_cast<T*>(pods[i]);
      if(!casted || casted->getName().compare(name) != 0)
	continue;
      
      passed.push_back(casted);
    }

    // Something has gone horribly wrong if two pods have the same name.
    PL_ASSERT(passed.size() < 2); 
    if(passed.size() == 1)
      return passed[0];
    else {
      PL_ABORT("Called getPod<T>(\"" << name << "\"), but no Pods of type T with this name were found." << std::endl
	       << "Typeid of T is: " << typeid(T).name());
    }
    return NULL;
  }

  template<typename T> T* Pipeline::getPod(const std::string& name) const
  {
    return pipeline::getPod<T>(name, pods_);
  }

  template<typename T> T Pipeline::getOutput(const std::string& pod_name, const std::string& outlet_name) const
  {
    return getPod(pod_name)->getOutlet(outlet_name)->pull<T>();
  }

  template<typename T> void Pipeline::getOutput(const std::string& pod_name,
						const std::string& outlet_name,
						T* dest) const
  {
    *dest = getPod(pod_name)->getOutlet(outlet_name)->pull<T>();
  }

  template<typename T> void Pipeline::setInput(const std::string& pod_name, T data)
  {
    getPod< EntryPoint<T> >(pod_name)->setData(data);
  }
  
} // namespace pipeline

#endif // PIPELINE_H
