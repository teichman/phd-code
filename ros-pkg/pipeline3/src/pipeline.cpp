#include <pipeline/pipeline.h>

using namespace std;

namespace pipeline {

  Pipeline::Pipeline(int num_threads) :
    num_threads_(num_threads),
    debug_(false),
    done_computation_(false),
    num_nodes_computing_(0),
    destructing_(false),
    cnf_(new ComputeNodeFactory)
  {
    pthread_mutex_init(&mutex_, NULL);
    pthread_cond_init(&queue_cv_, NULL);
    pthread_cond_init(&done_cv_, NULL);
    
    assertCompleteness();
    spawnThreadPool(num_threads_);
  }

  Pipeline::~Pipeline() {
    delete cnf_;

    killThreadPool();
    pthread_mutex_destroy(&mutex_);
    pthread_cond_destroy(&queue_cv_);
    pthread_cond_destroy(&done_cv_);

    for(size_t i = 0; i < nodes_.size(); ++i)
      delete nodes_[i];
  }

  void Pipeline::setDebug(bool debug)
  {
    debug_ = debug;
    for(size_t i = 0; i < nodes_.size(); ++i)
      nodes_[i]->debug_ = debug;
  }

  bool Pipeline::getDebug() const
  {
    return debug_;
  }
  
  void Pipeline::setNumThreads(int num_threads)
  {
    killThreadPool();
    spawnThreadPool(num_threads);
  }
  

  void Pipeline::assertNoDuplicates() const
  {
    // TODO: Add nicer error messages.

    // Check for duplicate node pointers.
    set<ComputeNode*> all;
    for(size_t i = 0; i < nodes_.size(); ++i)
      all.insert(nodes_[i]);
    ROS_ASSERT(all.size() == nodes_.size());

    // Check for duplicated names.
    set<string> names;
    for(size_t i = 0; i < nodes_.size(); ++i) {
      ROS_ASSERT(names.count(nodes_[i]->getName()) == 0);
      names.insert(nodes_[i]->getName());
    }
  }
  
  void Pipeline::addComponent(ComputeNode* node)
  {
    vector<ComputeNode*> component = getComponent(node);
    nodes_.insert(nodes_.end(), component.begin(), component.end());
    assertNoDuplicates();
  }

  bool Pipeline::trylock() {
    if(pthread_mutex_trylock(&mutex_) == EBUSY)
      return false;
    else
      return true;
  }
  
  void Pipeline::lock() {
    pthread_mutex_lock(&mutex_);
  }
  
  void Pipeline::unlock() {
    pthread_mutex_unlock(&mutex_);
  }
  
  void Pipeline::assertCompleteness() {
    queue<ComputeNode*> to_check;
    for(size_t i = 0; i < nodes_.size(); ++i) {
      if(nodes_[i]->upstream_.empty())
	to_check.push(nodes_[i]);
    }

    set<ComputeNode*> found;
    while(!to_check.empty()) {
      ComputeNode* active = to_check.front();
      to_check.pop();
      found.insert(active); //Won't insert duplicates.
      for(size_t i = 0; i < active->downstream_.size(); ++i) {
	to_check.push(active->downstream_[i]);
      }
    }

    assert(found.size() == nodes_.size());  
  }
  
  void Pipeline::flush() {
    num_nodes_computing_ = 0;
    for(size_t i = 0; i < nodes_.size(); ++i) { 
      nodes_[i]->flush();
      assert(!nodes_[i]->done_computation_);
    }
    done_computation_ = false;
  }

  void Pipeline::reset()
  {
    flush();
    for(size_t i = 0; i < nodes_.size(); ++i)
      nodes_[i]->reset();
  }

  string Pipeline::reportTiming() {
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
      names.push_back(nodes_[i]->getName());
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
    oss << "============================================================" << endl;
    
    return oss.str();
  }

  string sanitize(const string& name) {
    string sani = name.substr(0, name.find_first_of("_"));
    return sani;
  }
  
  string Pipeline::getGraphviz() const {
    ostringstream oss;
    oss << "digraph {" << endl;
    oss << endl;

    for(size_t i = 0; i < nodes_.size(); ++i) {
      oss << (uint64_t)nodes_[i] << " [label=\"" << sanitize(nodes_[i]->getName()) << "\"]" << endl;
    }
    oss << endl;

    for(size_t i = 0; i < nodes_.size(); ++i) {
      vector<ComputeNode*>& downstream = nodes_[i]->downstream_;
      for(size_t j = 0; j < downstream.size(); ++j) {
	oss << (uint64_t)nodes_[i] << "->" << (uint64_t)downstream[j] << endl;
      }
    }
    oss << endl;
    
    oss << "}" << endl;
    return oss.str();
  }
  
  void Pipeline::writeGraphviz(const string& filename) const {
    ofstream file;
    file.open(filename.c_str());
    assert(file);
    file << getGraphviz() << endl;
    file.close();
  }

  int Pipeline::switchComponent(ComputeNode* node, bool disabled)
  {
    // -- Ensure one of this Pipeline's nodes is the one being disabled.
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

  void Pipeline::enableAll()
  {
    for(size_t i = 0; i < nodes_.size(); ++i)
      nodes_[i]->disabled_ = false;
  }
  
  void Pipeline::disableAll()
  {
    for(size_t i = 0; i < nodes_.size(); ++i)
      nodes_[i]->disabled_ = true;
  }
  
  int Pipeline::disableComponent(ComputeNode* node)
  {
    return switchComponent(node, true);
  }

  int Pipeline::enableComponent(ComputeNode* node)
  {
    return switchComponent(node, false);
  }

  void Pipeline::spawnThreadPool(int num_threads) {
    assert(threads_.empty());
    threads_.resize(num_threads);
    for(size_t i=0; i<threads_.size(); ++i) {
      timeval start, end;
      gettimeofday(&start, NULL);
      pthread_create(&(threads_[i]), NULL, propagateComputation, (void*)this);
      gettimeofday(&end, NULL);
    }
  }

  void Pipeline::killThreadPool()
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

  void Pipeline::compute() {
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
  
  void Pipeline::registerCompleted(ComputeNode* node) { 
    for(size_t i = 0; i < node->downstream_.size(); ++i) {
      node->downstream_[i]->lock();
      if(node->downstream_[i]->ready()) {
	// Debugging.  TODO: remove.
	for(size_t j = 0; j < queue_.size(); ++j)
	  assert(queue_[j] != node->downstream_[i]);
	
	node->downstream_[i]->on_queue_ = true;
	queue_.push_back(node->downstream_[i]);
	pthread_cond_signal(&queue_cv_);
      }
      node->downstream_[i]->unlock();
    }
  }

  void Pipeline::run()
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
  
  void* propagateComputation(void *pipeline)
  {
    Pipeline& pl = *((Pipeline*) pipeline);
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
      for(size_t i = 0; i < active->downstream_.size(); ++i) {
	if(component.count(active->downstream_[i]) == 0)
	  to_check.push(active->downstream_[i]);
      }
      for(size_t i = 0; i < active->upstream_.size(); ++i) { 
	if(component.count(active->upstream_[i]) == 0)
	  to_check.push(active->upstream_[i]);
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

  void Pipeline::setFactory(ComputeNodeFactory* cnf)
  {
    if(cnf_)
      delete cnf_;
    cnf_ = cnf;
  }
  
  void Pipeline::serialize(std::ostream& out) const
  {
    ROS_ASSERT(cnf_);
    out << "Pipeline" << endl;
    out << "Serialization version 1" << endl;
    out << "Nodes (" << nodes_.size() << ")" << endl << endl;
    for(size_t i = 0; i < nodes_.size(); ++i)
      out << cnf_->serializeNode(nodes_[i]);
  }
  
  void Pipeline::deserialize(std::istream& in)
  {
    ROS_ASSERT(nodes_.empty());

    string buf;
    getline(in, buf);
    ROS_ASSERT(buf.compare("Pipeline") == 0);
    getline(in, buf); // serialization version
    in >> buf;
    ROS_ASSERT(buf.compare("Nodes") == 0);
    in >> buf;
    size_t num_nodes = atoi(buf.substr(1, buf.size() - 1).c_str());
    getline(in, buf);

    vector<string> input_lines;
    map<string, ComputeNode*> nodes; // Storage until we can get them hooked up.
    for(size_t i = 0; i < num_nodes; ++i) { 
      ComputeNode* node = cnf_->deserializeNode(in, &input_lines);
      ROS_ASSERT(nodes.count(node->getName()) == 0);
      nodes[node->getName()] = node;
      nodes_.push_back(node);
    }

    for(size_t i = 0; i < input_lines.size(); ++i) {
      istringstream iss(input_lines[i]);
      string input_node_name;
      string input_outlet_name;
      string output_node_name;
      string output_outlet_name;
      string buf;
      iss >> input_node_name;
      iss >> input_outlet_name;
      iss >> buf;
      iss >> output_node_name;
      iss >> output_outlet_name;

      ROS_ASSERT(nodes.count(input_node_name) == 1);
      ROS_ASSERT(nodes.count(output_node_name) == 1);
      nodes[input_node_name]->registerInput(input_outlet_name, nodes[output_node_name]->getOutlet(output_outlet_name));
    }

  }
  
} // namespace pipeline
