#include <asp/asp.h>

using namespace std;
using namespace pipeline;
using namespace Eigen;

namespace asp
{

  double sigmoid(double z)
  {
    return 1.0 / (1.0 + exp(-z));
  }
  
  Asp::Asp(int num_threads) :
    Pipeline(num_threads)
  {
    addPod(new EntryPoint<cv::Mat3b>("ImageEntryPoint"));
    addPod(new EntryPoint<cv::Mat1b>("SeedEntryPoint"));
    addPod(new NodePotentialAggregator("NodePotentialAggregator"));
    addPod(new EdgePotentialAggregator("EdgePotentialAggregator"));
    connect("ImageEntryPoint:Output -> NodePotentialAggregator:BackgroundImage");
    connect("ImageEntryPoint:Output -> EdgePotentialAggregator:BackgroundImage");
    addPod(new GraphcutsPod("GraphcutsPod"));
    connect("NodePotentialAggregator:Source -> GraphcutsPod:AggregatedSourcePotentials");
    connect("NodePotentialAggregator:Sink -> GraphcutsPod:AggregatedSinkPotentials");
    connect("EdgePotentialAggregator:Edge -> GraphcutsPod:AggregatedEdgePotentials");
    connect("ImageEntryPoint:Output -> GraphcutsPod:BackgroundImage");
    addPod(new SeedNPG("SeedNPG"));
    connect("ImageEntryPoint:Output -> SeedNPG:BackgroundImage");
    connect("SeedEntryPoint:Output -> SeedNPG:SeedImage");
    connect("SeedNPG:Source -> NodePotentialAggregator:UnweightedSource");
    connect("SeedNPG:Sink -> NodePotentialAggregator:UnweightedSink");
    addPod(new EdgeStructureGenerator("EdgeStructureGenerator"));
    setParam("EdgeStructureGenerator", "AxisAlignedGrid", true);
    setParam("EdgeStructureGenerator", "Web", false);
    connect("ImageEntryPoint:Output -> EdgeStructureGenerator:Image");
  }

  Model Asp::defaultModel() const
  {
    Model mod = model();
    mod.nweights_ = VectorXd::Zero(mod.nameMapping("nmap").size());
    mod.eweights_ = VectorXd::Zero(mod.nameMapping("emap").size());
    return mod;
  }

  Model Asp::model() const
  {
    Model mod;
    NodePotentialAggregator* npa = (NodePotentialAggregator*)pod("NodePotentialAggregator");
    npa->fillModel(&mod);
    EdgePotentialAggregator* epa = (EdgePotentialAggregator*)pod("EdgePotentialAggregator");
    epa->fillModel(&mod);
    return mod;
  }
  
  void Asp::setModel(const Model& mod)
  {
    NodePotentialAggregator* npa = (NodePotentialAggregator*)pod("NodePotentialAggregator");
    npa->setWeights(mod);
    EdgePotentialAggregator* epa = (EdgePotentialAggregator*)pod("EdgePotentialAggregator");
    epa->setWeights(mod);
  }
  
  void NodePotentialGenerator::initializeStorage()
  {
    cv::Mat3b img = pull<cv::Mat3b>("BackgroundImage");
    if(source_.rows() != img.rows || source_.cols() != img.cols)
      source_.resize(img.rows, img.cols);
    if(sink_.rows() != img.rows || sink_.cols() != img.cols)
      sink_.resize(img.rows, img.cols);

    source_.setZero();
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
    string raw_path = debugBasePath() + "-raw.png";
    cv::imwrite(raw_path, scaled_raw);
    cout << "Wrote node potential visualization to " << raw_path << endl;

    // -- Overlay.
    if(numIncoming("BackgroundImage") == 0)
      return;

    cv::Mat3b img = pull<cv::Mat3b>("BackgroundImage");
    cv::Mat3b vis;
    vis = img.clone();
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
    string overlay_path = debugBasePath() + "-overlay.png";
    cv::imwrite(overlay_path, scaled);
    cout << "Wrote node potential visualization to " << overlay_path << endl;
  }

  NameMapping NodePotentialAggregator::generateNameMapping() const
  {
    ROS_ASSERT(numIncoming("UnweightedSource") == numIncoming("UnweightedSink"));

    vector<string> source_names = upstreamOutputNames("UnweightedSource");
    vector<string> sink_names = upstreamOutputNames("UnweightedSink");
    ROS_ASSERT(source_names.size() == sink_names.size());
    for(size_t i = 0; i < source_names.size(); ++i)
      ROS_ASSERT(source_names[i] == sink_names[i]);

    NameMapping nmap;
    nmap.addNames(source_names);
    return nmap;
  }
  
  void NodePotentialAggregator::setWeights(Model model)
  {
    // Make the numbers in model match up with what we have here.
    NameMapping nmap = generateNameMapping();
    ROS_ASSERT(nmap.size() == (size_t)model.nweights_.rows());
    model.applyNameMapping("nmap", generateNameMapping());

    ROS_ASSERT(numIncoming("UnweightedSource") == numIncoming("UnweightedSink"));
    vector<string> names = upstreamOutputNames("UnweightedSource");
    for(size_t i = 0; i < names.size(); ++i)
      ROS_ASSERT(names[i] == model.nameMapping("nmap").toName(i));
  
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
    
    vector<const MatrixXd*> source;
    pull("UnweightedSource", &source);
    ROS_ASSERT((size_t)nweights_.rows() == source.size());
    for(size_t i = 0; i < source.size(); ++i)
      source_ += (*source[i]) * nweights_[i];
    
    vector<const MatrixXd*> sink;
    pull("UnweightedSink", &sink);
    ROS_ASSERT((size_t)nweights_.rows() == sink.size());
    for(size_t i = 0; i < sink.size(); ++i)
      sink_ += (*sink[i]) * nweights_[i];

    push<const MatrixXd*>("Source", &source_);
    push<const MatrixXd*>("Sink", &sink_);
  }

  void NodePotentialAggregator::debug() const
  {
    writeNodePotentialVisualization();
  }
  
  void EdgePotentialGenerator::initializeStorage(double reserve_per_node)
  {
    cv::Mat3b img = pull<cv::Mat3b>("BackgroundImage");
    int num_nodes = img.rows * img.cols;
    initializeSparseMat(num_nodes, num_nodes, reserve_per_node, &edge_);
  }

  void EdgePotentialGenerator::writeEdgePotentialVisualization() const
  {
    cv::Mat3b img = pull<cv::Mat3b>("BackgroundImage");
    cv::Mat3b vis = drawEdgeVisualization(img, edge_);
    double minval = std::numeric_limits<double>::max();
    double maxval = -std::numeric_limits<double>::max();
    for(int i = 0; i < edge_.rows(); ++i) {
      SparseMatrix<double, RowMajor>::InnerIterator it(edge_, i);
      for(; it; ++it) {
	minval = min(minval, it.value());
	maxval = max(maxval, it.value());
      }
    }
    cout << getName() << ": range of edge weights is " << minval << " to " << maxval << endl;
    string overlay_path = debugBasePath() + ".png";
    cv::imwrite(overlay_path, vis);
  }
  
  void EdgePotentialAggregator::compute()
  {
    initializeStorage();

    vector<const SparseMat*> unweighted;
    pull("UnweightedEdge", &unweighted);
    ROS_ASSERT((size_t)eweights_.rows() == unweighted.size());
    for(size_t i = 0; i < unweighted.size(); ++i) {
      edge_ += eweights_(i) * (*unweighted[i]);
    }

    push<const SparseMat*>("Edge", &edge_);
  }

  void EdgePotentialAggregator::debug() const
  {
    writeEdgePotentialVisualization();
  }

  NameMapping EdgePotentialAggregator::generateNameMapping() const
  {
    NameMapping emap;
    emap.addNames(upstreamOutputNames("UnweightedEdge"));
    return emap;
  }

  void EdgePotentialAggregator::setWeights(Model model)
  {
    // Make the numbers in model match up with what we have here.
    NameMapping emap = generateNameMapping();
    ROS_ASSERT(emap.size() == (size_t)model.eweights_.rows());
    model.applyNameMapping("emap", generateNameMapping());

    vector<string> names = upstreamOutputNames("UnweightedEdge");
    for(size_t i = 0; i < names.size(); ++i)
      ROS_ASSERT(names[i] == model.nameMapping("emap").toName(i));
  
    eweights_ = model.eweights_;
  }

  void EdgePotentialAggregator::fillModel(Model* model) const
  {
    model->applyNameMapping("emap", generateNameMapping());
    model->eweights_ = eweights_;
  }

  void GraphcutsPod::compute()
  {
    const MatrixXd* source;
    const MatrixXd* sink;
    const SparseMat* edge;
    pull("AggregatedSourcePotentials", &source);
    pull("AggregatedSinkPotentials", &sink);
    pull("AggregatedEdgePotentials", &edge);

    // TODO: Could probably do this just once.  Does it matter?
    cv::Mat3b img = pull<cv::Mat3b>("BackgroundImage");
    int num_nodes = img.rows * img.cols;
    int max_num_edges = param<int>("ExpectedNumEdges") * img.rows * img.cols;
    Graph3d graph(num_nodes, max_num_edges);
    graph.add_node(num_nodes);

    // -- Fill the graph with node potentials.
    ROS_ASSERT(source->rows() == sink->rows());
    ROS_ASSERT(source->cols() == sink->cols());
    for(int i = 0; i < source->rows(); ++i) {
      for(int j = 0; j < source->cols(); ++j) {
	int idx = i * source->cols() + j;
	graph.add_tweights(idx, source->coeffRef(i, j), sink->coeffRef(i, j));
      }
    }

    // -- Fill the graph with edge potentials.
    //    TODO: Should this use symmetric or asymmetric edge potentials?
    SparseMatrix<double, Eigen::RowMajor> trans(edge->transpose());  // Unfortunately, yes.
    SparseMatrix<double, Eigen::RowMajor> sym = (*edge + trans) / 2.0;
    for(int i = 0; i < sym.outerSize(); ++i) {
      for(SparseMatrix<double, RowMajor>::InnerIterator it(sym, i); it; ++it) {
	if(it.col() <= it.row())
	  continue;

	ROS_WARN_STREAM_COND(it.value() < 0, "Edgepot weighted sum is negative: " << it.value());
	ROS_FATAL_STREAM_COND(isnan(it.value()), "NaN in edgepot.");
	graph.add_edge(it.col(), it.row(), it.value(), it.value());
      }
    }

    // -- Compute the segmentation.
    graph.maxflow();

    // -- Pull it out from the maxflow library.
    if(seg_.rows != source->rows() || seg_.cols != source->cols())
      seg_ = cv::Mat1b(cv::Size(source->cols(), source->rows()));

    for(int y = 0; y < seg_.rows; ++y) {
      for(int x = 0; x < seg_.cols; ++x) {
	int idx = index(y, x, seg_.cols);

	if(graph.what_segment(idx, Graph3d::SINK) == Graph3d::SOURCE)
	  seg_(y, x) = 255;
	else if(graph.what_segment(idx, Graph3d::SOURCE) == Graph3d::SINK)
	  seg_(y, x) = 0;
	else
	  seg_(y, x) = 127;
      }
    }

    push<cv::Mat1b>("Segmentation", seg_);
  }

  void GraphcutsPod::debug() const
  {
    // -- Just the segmentation.
    string seg_path = debugBasePath() + "-segmentation-raw.png";
    cv::imwrite(seg_path, seg_);

    // -- Overlay.
    if(numIncoming("BackgroundImage") == 0)
      return;

    cv::Mat3b img = pull<cv::Mat3b>("BackgroundImage");
    cv::Mat3b vis;
    vis = img.clone();
    ROS_ASSERT(vis.rows > 0 && vis.cols > 0);
    visualizeSegmentation(seg_, img, vis);

    string overlay_path = debugBasePath() + "-segmentation-outline.png";
    cv::imwrite(overlay_path, vis);
  }

  void visualizeSegmentation(cv::Mat1b seg, cv::Mat3b img, cv::Mat3b vis)
  {
    // -- Dull the colors of the background.
    cv::Mat3b dull_bg = img.clone();
    for(int y = 0; y < seg.rows; ++y) { 
      for(int x = 0; x < seg.cols; ++x) { 
	if(seg(y, x) != 255) { 
	  dull_bg(y, x)[0] *= 0.5; 
	  dull_bg(y, x)[1] *= 0.5;
	  dull_bg(y, x)[2] *= 0.5;
	}
      }
    }

    // -- Add a red border around the object.    
    cv::Mat3b mask = dull_bg.clone();
    cv::Mat1b dilation;
    cv::dilate(seg, dilation, cv::Mat(), cv::Point(-1, -1), 4);
    for(int y = 0; y < seg.rows; ++y) 
      for(int x = 0; x < seg.cols; ++x)
	if(dilation(y, x) == 255 && seg(y, x) != 255)
	  mask(y, x) = cv::Vec3b(0, 0, 255);
    cv::addWeighted(dull_bg, 0.5, mask, 0.5, 0.0, vis);
  }

  void SeedNPG::compute()
  {
    cout << "SeedNPG::compute()" << endl;
    initializeStorage();
    
    cv::Mat1b seed = pull<cv::Mat1b>("SeedImage");
    for(int y = 0; y < seed.rows; ++y) {
      for(int x = 0; x < seed.cols; ++x) {
	if(seed(y, x) == 255)
	  source_(y, x) = 1;
	else if(seed(y, x) == 0)
	  sink_(y, x) = 1;
      }
    }

    push<const MatrixXd*>("Source", &source_);
    push<const MatrixXd*>("Sink", &sink_);
  }

  void SeedNPG::debug() const
  {
    cout << "SeedNPG::debug()" << endl;
    writeNodePotentialVisualization();
  }

  void EdgeStructureGenerator::compute()
  {
    // -- Initialize the structure.
    cv::Mat3b img = pull<cv::Mat3b>("Image");
    int num_nodes = img.rows * img.cols;
    double reserve = 0;
    if(param<bool>("AxisAlignedGrid"))
      reserve += 4;
    if(param<bool>("DiagonalGrid"))
      reserve += 4;
    if(param<bool>("Web"))
      reserve += param<int>("WebOutgoingPerPixel");
    initializeSparseMat(num_nodes, num_nodes, reserve, &structure_);

    // -- Generate edges.

    if(param<bool>("AxisAlignedGrid")) {
      int idx = 0;
      for(int y = 0; y < img.rows; ++y) {
      	for(int x = 0; x < img.cols; ++x, ++idx) {
	  if(x < img.cols - 1)
      	    structure_.insert(idx, index(y, x + 1, img.cols)) = 1;
      	  if(y < img.rows - 1)
      	    structure_.insert(idx, index(y + 1, x, img.cols)) = 1;
      	}
      }
    }
    if(param<bool>("DiagonalGrid")) {
      // It'd be (much?) faster to do this combined with the above.
      // Maybe make a separate matrix and add them at the end.
      // int idx = 0;
      // for(int y = 0; y < img.rows; ++y) {
      // 	for(int x = 0; x < img.cols; ++x, ++idx) {
      // 	  if(y < img.rows - 1) {
      // 	    if(x < img.cols - 1)
      // 	      structure_.insert(idx, index(y + 1, x + 1, img.cols)) = 1;
      // 	    if(x > 0)
      // 	      structure_.insert(idx, index(y + 1, x - 1, img.cols)) = 1;
      // 	  }
      // 	}
      // }
    }
    if(param<bool>("Web"))
      abort();

    push<const SparseMat*>("EdgeStructure", &structure_);
  }

  void EdgeStructureGenerator::debug() const
  {
    cout << "EdgeStructureGenerator: " << structure_.nonZeros() << " edges with average weight of nonzeros of " << structure_.sum() / (structure_.nonZeros()) << endl;
      
    
    cv::Mat3b img = pull<cv::Mat3b>("Image");
    cv::Mat3b vis = drawEdgeVisualization(img, structure_);
    string overlay_path = debugBasePath() + ".png";
    cv::imwrite(overlay_path, vis);
  }

  void initializeSparseMat(int width, int height, double reserve_per_node, SparseMat* mat)
  {
    if(mat->rows() != height || mat->cols() != width) { 
      *mat = SparseMat(width, height);
      mat->reserve((int)(reserve_per_node * width * height));
    }
    mat->setZero();
  }

  int sign(int x)
  {
    return (x > 0) - (x < 0);
  }

  void drawLine(cv::Point2d pt, const cv::Point2d& pt1, double potential, cv::Mat3b vis)
  {
    if(pt1.x < 0 || pt1.x >= vis.cols)
      return;
    if(pt1.y < 0 || pt1.y >= vis.rows)
      return;

    double max_x = max(pt.x, pt1.x);
    double max_y = max(pt.y, pt1.y);
    double min_x = min(pt.x, pt1.x);
    double min_y = min(pt.y, pt1.y);
    double dx = max_x - min_x;
    double dy = max_y - min_y;
    dx /= max(fabs(dx), fabs(dy));
    dy /= max(fabs(dx), fabs(dy));
    double x = max(0., min_x);
    double y = max(0., min_y);
    for(; x < max_x && y < max_y && x < vis.cols && y < vis.rows; x += dx, y += dy) {
      vis(y, x)[0] = vis(y, x)[0] * (1.0 - potential);
      vis(y, x)[1] = vis(y, x)[1] * (1.0 - potential);
      vis(y, x)[2] = vis(y, x)[2] * (1.0 - potential);
    }
  }

  cv::Mat3b drawEdgeVisualization(cv::Mat3b img, const SparseMat& edge)
  {
    ROS_ASSERT(img.rows > 0 && img.cols > 0);
    double scale = 7;
    cv::Size sz(img.cols * scale, img.rows * scale);
    cv::Mat3b vis;
    cv::resize(img, vis, sz, cv::INTER_NEAREST);

    SparseMat normalized = edge;
    double max = -std::numeric_limits<double>::max();
    double min =  std::numeric_limits<double>::max();
    for(int i = 0; i < normalized.rows(); ++i) {
      SparseMat::InnerIterator it(normalized, i);
      for(; it; ++it) {
	if(it.value() > max)
	  max = it.value();
	if(it.value() < min)
	  min = it.value();
      }
    }
    normalized /= max;

    // -- Draw non-zero edges.
    for(int y = 0; y < img.rows; ++y) {
      for(int x = 0; x < img.cols; ++x) {
	int idx0 = index(y, x, img.cols);
	// cout << "y: " << y << ", x: " << x << ", idx0: " << idx0 << endl;
	// cout << normalized.rows() << " x " << normalized.cols() << endl;

	SparseMat::InnerIterator it(normalized, idx0);
	for(; it; ++it) {
	  int idx1 = it.col();
	  int y1 = idx1 / img.cols;
	  int x1 = idx1 - y1 * img.cols;
	  //cout << "idx1: " << idx1 << ", y1: " << y1 << ", x1: " << x1 << ", it.value: " << it.value() << endl;

	  cv::Point pt(x*scale, y*scale);
	  cv::Point pt1(x1*scale, y1*scale);
	  //cout << "pt: " << pt << ", pt1: " << pt1 << endl;
	  drawLine(pt, pt1, it.value(), vis);
	}
      }
    }

    return vis;
  }
  
}  // namespace asp


