#include <pipeline/compute_node.h>

using namespace std;
using boost::any;

namespace pipeline
{

  Outlet::Outlet(std::string name, ComputeNode* node) :
    has_data_(false),
    name_(name),
    node_(node)
  {
  }
  
  ComputeNode::ComputeNode(std::string name, Params params) :
    debug_(false),
    disabled_(false),
    name_(name),
    params_(params),
    num_finished_upstream_(0),
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
    map<string, Outlet*>::iterator it;
    for(it = outputs_.begin(); it != outputs_.end(); ++it)
      delete it->second;
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
    cout << "  " << name_ << " has no display functionality." << endl;
  }

  void ComputeNode::reset() {
    flush();
    _reset();
  }

  void ComputeNode::_reset() {
    
  }
  
  void ComputeNode::flush() {
    done_computation_ = false;
    started_computation_ = false;
    time_msec_ = -1;
    num_finished_upstream_ = 0;
    assert(!on_queue_);

    //! Flush all the outputs, too.
    std::map<std::string, Outlet*>::iterator it;
    for(it = outputs_.begin(); it != outputs_.end(); ++it)
      it->second->flush();
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
      cerr << name_ << " is done computation, but its compute() function is being called!" << endl;
    assert(!done_computation_);
  
    started_computation_ = true;
  
    timeval start, end;
    gettimeofday(&start, NULL);
    _compute();
    gettimeofday(&end, NULL);
    done_computation_ = true;
    time_msec_ = (end.tv_sec - start.tv_sec) * 1000. + (end.tv_usec - start.tv_usec) / 1000.;

    if(debug_) {
      cout << "Displaying " << name_ << endl;
      cout << "  Computation took " << time_msec_ << " ms." << endl;
      display();
    }

    // -- Inform child nodes of completion.
    for(size_t i = 0; i < downstream_.size(); ++i) {
      downstream_[i]->lock();
      ++downstream_[i]->num_finished_upstream_;
      downstream_[i]->unlock();
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
  
  // string ComputeNode::getShortName() const {
  //   string ancestors = getGenealogy();
  //   uint64_t hash = hashDjb2(ancestors.c_str());
  //   ostringstream oss;
  //   oss << _getName() << "_" << hash;

  //   if(oss.str().length() >= getFullName().length())
  //     return getFullName();

  //   return oss.str();
  // }
  
  string ComputeNode::getRunName(int width) const
  {
    ostringstream oss;
    oss << setw(width) << setfill('0') << num_times_computed_ << "-" << name_;
    return oss.str();
  }
  
  bool ComputeNode::ready() {
    if(disabled_ || on_queue_ || done_computation_ || started_computation_)
      return false;
    
    if((size_t)num_finished_upstream_ == upstream_.size())
      return true;
    else
      return false;
  }

  void ComputeNode::declareOutput(std::string name)
  {
    ROS_ASSERT(outputs_.count(name) == 0);
    outputs_[name] = new Outlet(name, this);
  }

  void ComputeNode::declareInput(std::string name)
  {
    ROS_ASSERT(declared_inputs_.count(name) == 0);
    declared_inputs_.insert(name);
  }

  void ComputeNode::assertValid(std::string name) const
  {
    ROS_ASSERT(name.find(" ") == string::npos);
  }

  void ComputeNode::registerInput(std::string name, Outlet* outlet)
  {
    ROS_ASSERT(inputs_.count(name) == 0);
    ROS_ASSERT(declared_inputs_.count(name) == 1);
    inputs_[name] = outlet;
    upstream_.push_back(outlet->getNode());
    outlet->getNode()->downstream_.push_back(this);
  }

  std::string ComputeNode::printOutputs() const
  {
    ostringstream oss;
    std::map<std::string, Outlet*>::const_iterator it;
    for(it = outputs_.begin(); it != outputs_.end(); ++it)
      oss << "  " << it->first;
    return oss.str();
  }

  Outlet* ComputeNode::getOutlet(std::string name) const
  {
    ROS_ASSERT(outputs_.count(name) == 1);
    return outputs_.find(name)->second; 
  }
  
} // namespace pipeline
