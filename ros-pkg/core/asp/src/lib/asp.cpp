#include <asp/asp.h>

using namespace std;
using namespace pipeline;

namespace asp
{

  Asp::Asp(int num_threads) :
    Pipeline(num_threads)
  {
    addPod(new EntryPoint<cv::Mat3b>("ImageEntryPoint"));
    addPod(new NodePotentialAggregator("NodePotentialAggregator"));
    pod("NodePotentialAggregator")->registerInput("BackgroundImage", pod("ImageEntryPoint"), "Output");
  }
  
  void NodePotentialGenerator::initializeStorage()
  {
    cv::Mat3b img = pull<cv::Mat3b>("BackgroundImage");
    source_.resize(img.rows, img.cols);
    source_.setZero();
    sink_.resize(img.rows, img.cols);
    sink_.setZero();
  }
  
  void NodePotentialGenerator::writeNodePotentialVisualization() const
  {    
    // -- Just the potentials.
    cv::Mat3b raw;
    raw = cv::Mat3b(source_.rows(),
		    source_.cols(),
		    cv::Vec3b(0, 0, 0));
    
    for(int y = 0; y < raw.rows; ++y) { 
      for(int x = 0; x < raw.cols; ++x) {
	// -1 for bg, +1 for fg.
	double val = 2.0 * sigmoid(1.1 * (source_(y, x) - sink_(y, x))) - 1.0;
	val = min(0.9, max(-0.9, val));
	if(val < 0)
	  raw(y, x)[1] = 255 * -val;
	else
	  raw(y, x)[2] = 255 * val;	  
      }
    }

    double scale = 3;
    cv::Mat3b scaled_raw;
    cv::Size sz;
    sz.width = raw.cols * scale;
    sz.height = raw.rows * scale;
    cv::resize(raw, scaled_raw, sz);
    cv::imwrite(debugBasePath() + "-raw.png", scaled_raw);

    // -- Overlay.
    if(numIncoming("BackgroundImage") == 0)
      return;

    cv::Mat3b img = pull<cv::Mat3b>("BackgroundImage");
    cv::Mat3b vis;
    vis = background.clone();
    ROS_ASSERT(vis.rows > 0 && vis.cols > 0);
    for(int y = 0; y < vis.rows; ++y) { 
      for(int x = 0; x < vis.cols; ++x) {
	double val = sigmoid(5.0 * (source_(y, x) - sink_(y, x))); // 1.0 for foreground.
	val = min(0.9, max(0.1, val));
	vis(y, x)[0] = vis(y, x)[0] * val;
	vis(y, x)[1] = vis(y, x)[1] * val;
	vis(y, x)[2] = vis(y, x)[2] * val;
      }
    }

    cv::Mat3b scaled;
    sz.width = vis.cols * scale;
    sz.height = vis.rows * scale;
    cv::resize(vis, scaled, sz);
    cv::imwrite(debugBasePath() + "-overlay.png", scaled);
  }

  NameMapping NodePotentialAggregator::generateNameMapping() const
  {
    ROS_ASSERT(numIncoming("UnweightedSource") == numIncoming("UnweightedSink"));
    const vector<const Outlet*>& inputs = inputs_["UnweightedSource"];
    NameMapping nmap;
    for(size_t i = 0; i < inputs.size(); ++i) {
      string podname = inputs[i]->getPod()->getName();
      ROS_ASSERT(podname == inputs_["UnweightedSink"][i]->getPod()->getName());
      nmap.addName(podname);
    }
    return nmap;
  }
  
  void NodePotentialAggregator::setWeights(Model model)
  {
    // Make the numbers in model match up with what we have here.
    NameMapping nmap = generateNameMapping();
    ROS_ASSERT(nmap.size() == (size_t)model.nweights_.rows());
    model.applyNameMapping(generateNameMapping());

    ROS_ASSERT(numIncoming("UnweightedSource") == numIncoming("UnweightedSink"));
    const vector<const Outlet*>& inputs = inputs_["UnweightedSource"];
    ROS_ASSERT((size_t)model.nweights_.rows() == inputs.size());
    ROS_ASSERT((size_t)model.nweights_.rows() == model.nameMapping("nmap").size());
    for(size_t i = 0; i < inputs.size(); ++i)
      ROS_ASSERT(inputs[i]->getPod()->getName() == model.nameMapping("nmap").toName(i));
  
    nweights_ = model.nweights_;
  }

  void NodePotentialAggregator::fillModel(Model* model) const
  {
    model->applyNameMapping("nmap", generateNameMapping());
    model->nweights_ = nweights_;
  }

  void NodePotentialAggregator::compute()
  {
    initializeStorage();
    
    vector<const MatrixXf*> source;
    pull("UnweightedSource", &source);
    ROS_ASSERT((size_t)nweights_.rows() == source.size());
    for(size_t i = 0; i < source.size(); ++i)
      source_ += (*source[i]) * nweights_[i];
    
    vector<const MatrixXf*> sink;
    pull("UnweightedSink", &sink);
    ROS_ASSERT((size_t)nweights_.rows() == sink.size());
    for(size_t i = 0; i < sink.size(); ++i)
      sink_ += (*sink[i]) * nweights_[i];

    push<const MatrixXf*>("Source", &source_);
    push<const MatrixXf*>("Sink", &sink_);
  }

  void NodePotentialAggregator::debug()
  {
    writeNodePotentialVisualization();
  }

}  // namespace asp
