#include <iostream>
#include <pipeline2/pipeline2.h>
#include <map>

using namespace std;
using namespace Eigen;

namespace pipeline2 {

  ComputeNode::ComputeNode() :
    debug_(false),
    disabled_(false),
    num_finished_inputs_(0),
    done_computation_(false),
    started_computation_(false),
    on_queue_(false),
    mutex_(pthread_mutex_t()),
    time_msec_(-1),
    num_times_computed_(0)
  {
  }

  ComputeNode::~ComputeNode()
  {
  }
  
  bool ComputeNode::trylock() {
    if(pthread_mutex_trylock(&mutex_) == EBUSY)
      return false;
    else
      return true;
  }
  
  void ComputeNode::lock() {
    pthread_mutex_lock(&mutex_);
  }
  
  void ComputeNode::unlock() {
    pthread_mutex_unlock(&mutex_);
  }
  
  double ComputeNode::getComputationTime() const {
    assert(done_computation_);
    return time_msec_;
  }

  int ComputeNode::getNumTimesComputed() const
  {
    return num_times_computed_;
  }
  
  void ComputeNode::display() const {
    _display();
  }
  
  void ComputeNode::_display() const {
    cout << "  " << getFullName() << " has no display functionality." << endl;
  }

  void ComputeNode::reset() {
    flush();
    _reset();
  }

  void ComputeNode::_reset() {
    
  }
  
  void ComputeNode::flush() {
    _flush();
    done_computation_ = false;
    started_computation_ = false;
    time_msec_ = -1;
    num_finished_inputs_ = 0;
    assert(!on_queue_);
  }
  
  void ComputeNode::compute() {
    // -- Extra debugging.  Remove this.
//     int val = pthread_mutex_trylock(&mutex_);
//     if(val != 0) {
//       cout << "Starting computation, but node is locked.  trylock return val: " << val << endl;
//       cout << "EBUSY: " << EBUSY << ", EINVAL: " << EINVAL << ", EAGAIN: " << EAGAIN << endl;
//       assert(val == 0);
//     }

    assert(on_queue_);
    on_queue_ = false;

    //assert(!trylock());
    assert(!started_computation_);
    if(done_computation_)
      cerr << getFullName() << " is done computation, but its compute() function is being called!" << endl;
    assert(!done_computation_);
  
    started_computation_ = true;
  
    timeval start, end;
    gettimeofday(&start, NULL);
    _compute();
    gettimeofday(&end, NULL);
    done_computation_ = true;
    time_msec_ = (end.tv_sec - start.tv_sec) * 1000. + (end.tv_usec - start.tv_usec) / 1000.;

    if(debug_) {
      // cout << "Displaying " << getFullName() << endl;
      // cout << "  Computation took " << time_msec_ << " ms." << endl;
      display();
    }

    // -- Inform child nodes of completion.
    for(size_t i = 0; i < outputs_.size(); ++i) {
      outputs_[i]->lock();
      ++outputs_[i]->num_finished_inputs_;
      outputs_[i]->unlock();
    }

    ++num_times_computed_;
  }

  uint64_t hashDjb2(const char *str) {
    uint64_t hash = 5381;
    int c;

    // See http://www.cse.yorku.ca/~oz/hash.html.
    while((c = *str++))
      hash = ((hash << 5) + hash) + c; /* hash * 33 + c */
  
    return hash;
  }
  
  string ComputeNode::getShortName() const {
    string ancestors = getGenealogy();
    uint64_t hash = hashDjb2(ancestors.c_str());
    ostringstream oss;
    oss << _getName() << "_" << hash;

    if(oss.str().length() >= getFullName().length())
      return getFullName();

    return oss.str();
  }
  

  string ComputeNode::getGenealogy() const {
    if(inputs_.size() == 0)
      return string("");
    
    ostringstream oss;
    oss << "(";
    for(size_t i = 0; i < inputs_.size(); ++i) { 
      oss << inputs_[i]->getFullName();
      if(i < inputs_.size() - 1)
        oss << "&&";
    }
    oss << ")=>";
    return oss.str();
  }

  string ComputeNode::getRunName(int width) const
  {
    ostringstream oss;
    oss << setw(width) << setfill('0') << num_times_computed_ << "-" << _getName();
    return oss.str();
  }
  
  string ComputeNode::getFullName() const {
    ostringstream oss;
    oss << getGenealogy() << _getName();
    return oss.str();
  }

  void ComputeNode::registerInput(ComputeNode* input) {
    inputs_.push_back(input);
    input->outputs_.push_back(this);
  }

  bool ComputeNode::ready() {
    if(disabled_ || on_queue_ || done_computation_ || started_computation_)
      return false;
    
    if((size_t)num_finished_inputs_ == inputs_.size())
      return true;
    else
      return false;
  }

  Pipeline2::Pipeline2(int num_threads, const vector<ComputeNode*>& nodes) :
    nodes_(nodes),
    num_threads_(num_threads),
    debug_(false),
    done_computation_(false),
    num_nodes_computing_(0),
    destructing_(false)
  {
    pthread_mutex_init(&mutex_, NULL);
    pthread_cond_init(&queue_cv_, NULL);
    pthread_cond_init(&done_cv_, NULL);
    
    queue_.reserve(nodes.size());
    
    assertCompleteness();
    spawnThreadPool(num_threads_);
  }

  Pipeline2::~Pipeline2() {
    killThreadPool();

    pthread_mutex_destroy(&mutex_);
    pthread_cond_destroy(&queue_cv_);
    pthread_cond_destroy(&done_cv_);

    for(size_t i = 0; i < nodes_.size(); ++i)
      delete nodes_[i];
  }

  void Pipeline2::setNodes(const vector<ComputeNode*>& nodes) {
    //assert(!computing());
    nodes_ = nodes;
  }

  void Pipeline2::setDebug(bool debug)
  {
    debug_ = debug;
    for(size_t i = 0; i < nodes_.size(); ++i)
      nodes_[i]->debug_ = debug;

    // User must set number of threads himself.
    // You might want multithreading even in debug mode.
    // killThreadPool();
    // if(debug_)
    //   spawnThreadPool(1);
    // else
    //   spawnThreadPool(num_threads_);
  }

  bool Pipeline2::getDebug() const
  {
    return debug_;
  }
  
  void Pipeline2::setNumThreads(int num_threads)
  {
    killThreadPool();
    spawnThreadPool(num_threads);
  }
  
  void Pipeline2::addComponent(ComputeNode* node)
  {
    vector<ComputeNode*> component = getComponent(node);
    nodes_.insert(nodes_.end(), component.begin(), component.end());

    // -- Check that no duplicates were added.
    set<ComputeNode*> all;
    for(size_t i = 0; i < nodes_.size(); ++i)
      all.insert(nodes_[i]);

    ROS_ASSERT(all.size() == nodes_.size());
  }

  bool Pipeline2::trylock() {
    if(pthread_mutex_trylock(&mutex_) == EBUSY)
      return false;
    else
      return true;
  }
  
  void Pipeline2::lock() {
    pthread_mutex_lock(&mutex_);
  }
  
  void Pipeline2::unlock() {
    pthread_mutex_unlock(&mutex_);
  }
  
  void Pipeline2::assertCompleteness() {
    queue<ComputeNode*> to_check;
    for(size_t i = 0; i < nodes_.size(); ++i) {
      if(nodes_[i]->inputs_.empty())
        to_check.push(nodes_[i]);
    }

    set<ComputeNode*> found;
    while(!to_check.empty()) {
      ComputeNode* active = to_check.front();
      to_check.pop();
      found.insert(active); //Won't insert duplicates.
      for(size_t i = 0; i < active->outputs_.size(); ++i) {
        to_check.push(active->outputs_[i]);
      }
    }

    assert(found.size() == nodes_.size());  
  }
  
  void Pipeline2::flush() {
    num_nodes_computing_ = 0;
    for(size_t i = 0; i < nodes_.size(); ++i) { 
      nodes_[i]->flush();
      assert(!nodes_[i]->done_computation_);
    }
    done_computation_ = false;
  }

  void Pipeline2::reset()
  {
    flush();
    for(size_t i = 0; i < nodes_.size(); ++i)
      nodes_[i]->reset();
  }

  string Pipeline2::reportSlowestPath()
  {
    // -- Check for duplicated node.
    set<ComputeNode*> all;
    for(size_t i = 0; i < nodes_.size(); ++i)
      all.insert(nodes_[i]);
    //ROS_DEBUG_STREAM("Sizes: " << nodes_.size() << " " << all.size());
    ROS_ASSERT(nodes_.size() == all.size());
      
    
    map<ComputeNode*, double> times;
    map<ComputeNode*, ComputeNode*> backptrs;

    // -- Initialize with entry points.
    queue<ComputeNode*> to_check;
    set<ComputeNode*> marked;
    for(size_t i = 0; i < nodes_.size(); ++i) { 
      if(nodes_[i]->inputs_.empty()) { 
        to_check.push(nodes_[i]);
        marked.insert(nodes_[i]);
        backptrs[nodes_[i]] = NULL;
        times[nodes_[i]] = nodes_[i]->getComputationTime();
        ROS_ASSERT(times.count(nodes_[i]) == 1);
      }
    }

    // -- Propagate max path times through the graph.
    while(!to_check.empty()) {
      // Check parents.
      ComputeNode* active = to_check.front();
      to_check.pop();
      
      double max = -std::numeric_limits<double>::max();
      for(size_t i = 0; i < active->inputs_.size(); ++i) {
        ROS_ASSERT(marked.count(active->inputs_[i]));
        ROS_ASSERT(times.count(active->inputs_[i]));
        double val = times[active->inputs_[i]] + active->getComputationTime();
        if(val > max) { 
          max = val;
          times[active] = val;
          backptrs[active] = active->inputs_[i];
        }
      }
      
      // Add children.
      for(size_t i = 0; i < active->outputs_.size(); ++i) {
        ComputeNode* downstream = active->outputs_[i];
        if(marked.count(downstream)) {
          continue;
        }
        bool all_parents_done = true;
        for(size_t j = 0; j < downstream->inputs_.size(); ++j) {
          if(!times.count(downstream->inputs_[j])) { 
            all_parents_done = false;
            break;
          }
        }

        if(all_parents_done) { 
          to_check.push(downstream);
          ROS_ASSERT(marked.count(downstream) == 0);
          marked.insert(downstream);
        }


      }
    }

    if(marked.size() != nodes_.size()) { 
      for(size_t i = 0; i < nodes_.size(); ++i)
        if(marked.count(nodes_[i]) == 0)
          cout << "Node not in marked: " << nodes_[i]->_getName() << endl;

      cout << "Error in pipeline2::reportSlowestPath.  marked.size() == "
           << marked.size() << ", nodes_.size() == " << nodes_.size() << endl;
      abort();
    }
    
    // -- Find the endpoint with longest time.
    double max = 0;
    ComputeNode* active = NULL;
    map<ComputeNode*, double>::iterator it;
    for(it = times.begin(); it != times.end(); ++it) {
      //cout << it->first->_getName() << ": " << it->second << endl;
      if(it->second > max) {
        max = it->second;
        active = it->first;
      }
    }

    // -- Trace the path back to the entry point.
    vector<ComputeNode*> path;
    while(active) {
      path.push_back(active);
      active = backptrs[active];
    }

    ostringstream oss;
    oss << "Slowest path (" << max << " ms): " << endl;
    for(int i = (int)path.size() - 1; i >= 0; --i) {
      oss << "  " << fixed << setprecision(2) << setw(8) << times[path[i]]
          << "\t" << fixed << setprecision(2) << setw(8) << path[i]->getComputationTime()
          << "\t" << path[i]->_getName() << endl;
    }
    return oss.str();
  }
  
  string Pipeline2::reportTiming() {
    assert(done_computation_);
    
    ostringstream oss;
    oss << "============================================================" << endl;
    oss << "Pipeline timing report" << endl;
    oss << "Time (ms) \tNode name" << endl;
    oss << "------------------------------------------------------------" << endl;
    vector<string> names;
    vector<double> times;
    vector< pair<double, size_t> > index;
    double total_time = 0;
    for(size_t i = 0; i < nodes_.size(); ++i) {
      names.push_back(nodes_[i]->_getName());
      times.push_back(nodes_[i]->getComputationTime());
      index.push_back(pair<double, size_t>(times.back(), i));
      total_time += times.back();
    }
    sort(index.begin(), index.end(), greater< pair<double, size_t> >());
    
    for(size_t i = 0; i < index.size(); ++i) {
      size_t idx = index[i].second;
      oss << times[idx] << "\t" << "\t" << names[idx] << endl;
    }
    oss << "Number of threads:\t" << threads_.size() << endl;
    oss << "Sum of node compute times:\t" << total_time << " ms." << endl;
    oss << "Start-to-finish wall time:\t" << time_msec_ << " ms." << endl;
    oss << reportSlowestPath() << endl;
    oss << "============================================================" << endl;
    
    return oss.str();
  }

  string sanitize(const string& name) {
    string sani = name.substr(0, name.find_first_of("_"));
    return sani;
  }
  
  string Pipeline2::getGraphviz() const {
    ostringstream oss;
    oss << "digraph {" << endl;
    oss << endl;

    for(size_t i = 0; i < nodes_.size(); ++i) {
      oss << (uint64_t)nodes_[i] << " [label=\"" << sanitize(nodes_[i]->_getName()) << "\"]" << endl;
    }
    oss << endl;

    for(size_t i = 0; i < nodes_.size(); ++i) {
      vector<ComputeNode*>& outputs = nodes_[i]->outputs_;
      for(size_t j = 0; j < outputs.size(); ++j) {
        oss << (uint64_t)nodes_[i] << "->" << (uint64_t)outputs[j] << endl;
      }
    }
    oss << endl;
    
    oss << "}" << endl;
    return oss.str();
  }
  
  void Pipeline2::writeGraphviz(const string& filename) const {
    ofstream file;
    file.open(filename.c_str());
    assert(file);
    file << getGraphviz() << endl;
    file.close();
  }

  int Pipeline2::switchComponent(ComputeNode* node, bool disabled)
  {
    // -- Ensure one of this Pipeline2's nodes is the one being disabled.
    assert(node);
    bool valid = false;
    for(size_t i = 0; i < nodes_.size(); ++i) {
      if(node == nodes_[i])
        valid = true;
    }
    assert(valid);

    vector<ComputeNode*> component = getComponent(node);
    for(size_t i = 0; i < component.size(); ++i)
      component[i]->disabled_ = disabled;

    return component.size();
  }

  void Pipeline2::enableAll()
  {
    for(size_t i = 0; i < nodes_.size(); ++i)
      nodes_[i]->disabled_ = false;
  }
  
  void Pipeline2::disableAll()
  {
    for(size_t i = 0; i < nodes_.size(); ++i)
      nodes_[i]->disabled_ = true;
  }
  
  int Pipeline2::disableComponent(ComputeNode* node)
  {
    return switchComponent(node, true);
  }

  int Pipeline2::enableComponent(ComputeNode* node)
  {
    return switchComponent(node, false);
  }

  void Pipeline2::spawnThreadPool(int num_threads) {
    assert(threads_.empty());
    threads_.resize(num_threads);
    for(size_t i=0; i<threads_.size(); ++i) {
      timeval start, end;
      gettimeofday(&start, NULL);
      pthread_create(&(threads_[i]), NULL, propagateComputation, (void*)this);
      gettimeofday(&end, NULL);
    }
  }

  void Pipeline2::killThreadPool()
  {
    lock();
    destructing_ = true;
    pthread_cond_broadcast(&queue_cv_);
    unlock();
    
    for(size_t i = 0; i < threads_.size(); ++i)
      pthread_join(threads_[i], NULL);

    lock();
    threads_.clear();
    destructing_ = false;
    unlock();
  }

  void Pipeline2::compute() {
    lock();
    HighResTimer hrt;
    hrt.start();
    assert(!done_computation_);
    assert(queue_.empty());
    assert(!threads_.empty());

    // -- Find all ready nodes and put them into the queue.
    for(size_t i = 0; i < nodes_.size(); ++i) {
      nodes_[i]->lock();
      if(nodes_[i]->ready()) {
        nodes_[i]->on_queue_ = true;
        queue_.push_back(nodes_[i]);
        pthread_cond_signal(&queue_cv_);
      }
      nodes_[i]->unlock();
    }
    assert(!queue_.empty());

    // -- Wait for the worker nodes to complete.
    pthread_cond_wait(&done_cv_, &mutex_);
    
    done_computation_ = true;
    hrt.stop();
    time_msec_ = hrt.getMilliseconds();
    unlock();
  }
  
  void Pipeline2::registerCompleted(ComputeNode* node) { 
    for(size_t i = 0; i < node->outputs_.size(); ++i) {
      node->outputs_[i]->lock();
      if(node->outputs_[i]->ready()) {
        // Debugging.  TODO: remove.
        for(size_t j = 0; j < queue_.size(); ++j)
          assert(queue_[j] != node->outputs_[i]);
        
        node->outputs_[i]->on_queue_ = true;
        queue_.push_back(node->outputs_[i]);
        pthread_cond_signal(&queue_cv_);
      }
      node->outputs_[i]->unlock();
    }
  }

  void Pipeline2::run()
  {
    lock();
    while(true) {
      if(destructing_) { 
        unlock();
        break;
      }
            
      if(queue_.empty()) {
        if(num_nodes_computing_ == 0)
          pthread_cond_signal(&done_cv_);
        
        pthread_cond_wait(&queue_cv_, &mutex_);
      }

      // pthread signal might awaken more than one thread.
      if(!queue_.empty()) {
        ComputeNode* node = queue_.back();
        queue_.pop_back();

        // Debugging. TODO: remove.
        assert(node->on_queue_);
        for(size_t i = 0; i < queue_.size(); ++i)
          assert(queue_[i] != node);

        ++num_nodes_computing_;
        unlock();

        node->lock();
        node->compute();
        node->unlock();

        lock();
        registerCompleted(node);
        --num_nodes_computing_;
      }
    }
  }
  
  void* propagateComputation(void *pipeline2)
  {
    Pipeline2& pl = *((Pipeline2*) pipeline2);
    pl.run();
    return NULL;
  }

  vector<ComputeNode*> getComponent(ComputeNode* node)
  {
    queue<ComputeNode*> to_check;
    to_check.push(node);

    set<ComputeNode*> component;
    while(!to_check.empty()) {
      ComputeNode* active = to_check.front();
      to_check.pop();
      component.insert(active); //Won't insert duplicates.
      for(size_t i = 0; i < active->outputs_.size(); ++i) {
        if(component.count(active->outputs_[i]) == 0)
          to_check.push(active->outputs_[i]);
      }
      for(size_t i = 0; i < active->inputs_.size(); ++i) { 
        if(component.count(active->inputs_[i]) == 0)
          to_check.push(active->inputs_[i]);
      }
    }

    vector<ComputeNode*> vec;
    vec.reserve(component.size());
    set<ComputeNode*>::const_iterator it;
    for(it = component.begin(); it != component.end(); ++it) {
      vec.push_back(*it);
    }
    return vec;
  }
  
} // namespace pipeline2
