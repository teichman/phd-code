#include <dst/volume_segmenter.h>

#define NUM_THREADS (getenv("NUM_THREADS") ? atoi(getenv("NUM_THREADS")) : 40)

using namespace std;
using namespace Eigen;
namespace gc = graphcuts;

namespace dst
{

  VolumeSegmenter::VolumeSegmenter(const Eigen::VectorXd& weights) :
    weights_(weights)
  {
  }
  
  void VolumeSegmenter::segment(KinectSequence::ConstPtr seq,
				std::vector<cv::Mat1b>* predictions) const
  {
    ROS_ASSERT(predictions->empty());
    gc::PotentialsCache::Ptr pc(new gc::PotentialsCache);
    cacheSequence(seq, pc);
        
    // -- Run the segmenter and create output labels.
    gc::MaxflowInference mfi(weights_);
    VectorXi seg(seq->totalPixels());
    cout << "Running graph cuts segmentation." << endl;
    mfi.segment(pc, &seg);
    for(size_t i = 0; i < seq->images_.size(); ++i) {
      cv::Mat1b segimg(seq->images_[i].size(), 127);
      for(int y = 0; y < segimg.rows; ++y) {
	for(int x = 0; x < segimg.cols; ++x) {
	  int idx = i * seq->pixelsPerFrame() + y * segimg.cols + x;
	  ROS_ASSERT(idx < seg.rows());
	  if(seg(idx) == 1)
	    segimg(y, x) = 255;
	}
      }
      predictions->push_back(segimg);
    }
  }

  void VolumeSegmenter::addDenseInterFrameEdges(int frame_id,
						int epot_id,
						int nodes_per_frame,
						int total_nodes,
						const KinectCloud& pcd,
						KdTree::Ptr prev_kdtree,
						gc::PotentialsCache* pc) const
  {
    if(!prev_kdtree)
      return;
    
    ROS_ASSERT(pcd.isOrganized());

    double radius = 0.05;
    double sigma_color = 10.0;
    double sigma_dist = 2.5;

    DynamicSparseMatrix<double, RowMajor> storage(total_nodes, total_nodes);
    
    vector<int> indices;
    vector<float> distances;
    const KinectCloud& prev_pcd = *prev_kdtree->getInputCloud();
    ROS_ASSERT(prev_pcd.isOrganized());
    ROS_ASSERT(pcd.size() == (size_t)nodes_per_frame);
    ROS_ASSERT(prev_pcd.size() == (size_t)nodes_per_frame);
    for(size_t i = 0; i < pcd.size(); ++i) {
      if(isnan(pcd[i].x))
	continue;
      
      indices.clear();
      distances.clear();
      prev_kdtree->radiusSearch(pcd[i], radius, indices, distances);
      for(size_t j = 0; j < indices.size(); ++j) {
	if(rand() % 7 != 0)
	  continue;
	double dist = distances[j];
	const Point& prev_pt = prev_pcd[indices[j]];
	double dr = pcd[i].r - prev_pt.r;
	double dg = pcd[i].g - prev_pt.g;
	double db = pcd[i].b - prev_pt.b;
    	double dcolor = sqrt(dr*dr + dg*dg + db*db);
	double strength = exp(-dist / sigma_dist - dcolor / sigma_color);
	int from = (frame_id - 1) * nodes_per_frame + indices[j];
	int to = frame_id * nodes_per_frame + i;
	// cout << "Adding edge from " << from << " to " << to
	//      << ".  This call includes edges from [" << (frame_id-1) * nodes_per_frame
	//      << " " << frame_id * nodes_per_frame << "] to ["
	//      << frame_id * nodes_per_frame << " " << (frame_id + 1) * nodes_per_frame << "]" << endl;
	storage.coeffRef(from, to) = strength;
      }
    }

    // Convert to SparseMatrix.
    pc->edge_[epot_id] += gc::SparseMat(storage);
  }
  
  void VolumeSegmenter::addIntraFrameEdges(int offset,
					   FramePotentialsCache::ConstPtr cache,
					   gc::PotentialsCache* pc) const
  {
    for(size_t i = 0; i < cache->edge_potentials_.size(); ++i) {
      const gc::SparseMat& old = cache->edge_potentials_[i];
      gc::SparseMat& vol = pc->edge_[i];
      for(int j = 0; j < old.outerSize(); ++j) {
	vol.startVec(j + offset);
	for(gc::SparseMat::InnerIterator it(old, j); it; ++it)
	  vol.insertBack(j + offset, it.col() + offset) = it.value();
      }
    }
  }

  void VolumeSegmenter::addFlowEdges(int frame_id,
				     int epot_id,
				     int nodes_per_frame,
				     OpticalFlowNode::Output flow,
				     gc::PotentialsCache* pc) const
  {
    if(frame_id == 0)
      return;

    int cols = flow.img_.cols;
    vector< pair<int, int> > index;  // start node id, vector id.
    for(size_t i = 0; i < flow.prev_points_->size(); ++i) {
      if(flow.edge_scores_->at(i) > 0.2)
	continue;
      const cv::Point2i& prev = flow.prev_points_->at(i);
      int start_node_id = (frame_id - 1) * nodes_per_frame + prev.y * cols + prev.x;
      index.push_back(pair<int, int>(start_node_id, i));
    }

    // Get them in order of node id so we can add them to the sparse matrix.
    sort(index.begin(), index.end());

    gc::SparseMat& epot = pc->edge_[epot_id];
    size_t idx = 0;
    for(int i = (frame_id - 1) * nodes_per_frame; i < frame_id * nodes_per_frame; ++i) {
      epot.startVec(i);
      if(index[idx].first == i) {
	int vid = index[idx].second;
	const cv::Point2i& curr = flow.points_->at(vid);
	int end_node_id = frame_id * nodes_per_frame + curr.y * cols + curr.x;
	epot.insertBack(i, end_node_id) = 1.0 - flow.edge_scores_->at(vid);
	++idx;
      }
    }
  }

  cv::Mat1b visualizeEdges(const gc::SparseMat& edges)
  {
    double scale = 0.001;
    int rows = edges.rows() * scale;
    int cols = edges.cols() * scale;
    cv::Mat1b vis(rows, cols);
    vis = 0;

    int y = 0;
    int x = 0;
    for(int i = 0; i < edges.outerSize(); ++i) {
      for(gc::SparseMat::InnerIterator it(edges, i); it; ++it) {
	y = it.row() * scale;
	x = it.col() * scale;
	vis(y, x) = 255;
      }
    }

    return vis;
  }

  void VolumeSegmenter::cacheSequence(KinectSequence::ConstPtr seq,
				      gc::PotentialsCache::Ptr cache,
				      gc::VecXiPtr labels) const
  {
    ROS_ASSERT(cache->edge_.empty());
    
    // -- Set up potentials storage.
    gc::PotentialsCache& pc = *cache;
    int nodes_per_frame = seq->images_[0].rows * seq->images_[0].cols;
    int total_nodes = seq->images_.size() * nodes_per_frame;
    SegmentationPipeline sp(NUM_THREADS);
    int num_intra_epots = sp.getEdgeWeights().rows();
    for(int i = 0; i < num_intra_epots; ++i)
      pc.edge_.push_back(gc::SparseMat(total_nodes, total_nodes));
    int num_inter_epots = 2;
    for(int i = 0; i < num_inter_epots; ++i)
      pc.edge_.push_back(gc::SparseMat(total_nodes, total_nodes));
    
    // Add the intercept term.
    pc.sink_.push_back(VectorXd::Ones(total_nodes));
    pc.source_.push_back(VectorXd::Zero(total_nodes));
    
    // -- Initialize with large node potentials for the seed frame.
    VectorXd seed_src = VectorXd::Zero(total_nodes);
    VectorXd seed_snk = VectorXd::Zero(total_nodes);
    cv::Mat1b seed = seq->seed_images_[0];
    for(int y = 0; y < seed.rows; ++y) {
      for(int x = 0; x < seed.cols; ++x) {
	int idx = y * seed.cols + x;
	if(seed(y, x) == 255)
	  seed_src(idx) = 10000;
	else if(seed(y, x) == 0)
	  seed_snk(idx) = 10000;
      }
    }
    pc.source_.push_back(seed_src);
    pc.sink_.push_back(seed_snk);
        
    // -- Run the feed-forward segmentation pipeline on the sequence.
    //    Fill volume edge potentials.
    sp.verbose_ = false;
    cv::Mat1b pred(seq->images_[0].size(), 127);
    cv::Mat1b empty_seed(seq->images_[0].size(), 127);
    for(size_t i = 0; i < seq->images_.size(); ++i) {
      if(i == 0) {
	sp.run(seq->seed_images_[i],
	       seq->images_[i],
	       seq->pointclouds_[i],
	       cv::Mat3b(),
	       cv::Mat1b(),
	       KinectCloud::Ptr(),
	       pred,
	       KinectCloud::Ptr());
      }
      else {
	sp.run(empty_seed,
	       seq->images_[i],
	       seq->pointclouds_[i],
	       seq->images_[i-1],
	       pred,
	       seq->pointclouds_[i-1],
	       pred,
	       KinectCloud::Ptr());
      }

      // Generate intra-frame edges in the volume using the feed-forward method's edges.
      FramePotentialsCache::Ptr fc = sp.getFrameCache();
      addIntraFrameEdges(i * nodes_per_frame, fc, &pc);

      // Generate inter-frame edges in the volume using optical flow.
      OpticalFlowNode* ofn = sp.pipeline_.getNode<OpticalFlowNode>();
      OpticalFlowNode::Output flow = ofn->optflow_otl_.pull();
      addFlowEdges(i, fc->edge_potentials_.size(),
      		   nodes_per_frame, flow, &pc);

      SceneAlignmentNode* san = sp.pipeline_.getNode<SceneAlignmentNode>();
      KinectCloud::ConstPtr pcd = san->transformed_otl_.pull();
      //KinectCloud::ConstPtr pcd = ktn->kdtree_otl_.pull().current_pcd_;
      KdTreeNode* ktn = sp.pipeline_.getNode<KdTreeNode>();
      KdTree::Ptr prev_kdtree = ktn->kdtree_otl_.pull().previous_kdtree_;

      if(pcd) {
	ROS_ASSERT(!getenv("MASK"));
	cout << "Adding dense inter-frame edges for frame " << i << endl;
	addDenseInterFrameEdges(i, fc->edge_potentials_.size() + 1,
				nodes_per_frame, total_nodes,
				*pcd, prev_kdtree, &pc);
      }
    }

    // -- Finalize all the edge potentials.
    for(size_t i = 0; i < pc.edge_.size(); ++i) { 
      pc.edge_[i].finalize();
      // cv::imshow("Edges", visualizeEdges(pc.edge_[i]));
      // cv::waitKey();
    }
    pc.symmetrizeEdges();

    // -- Display the sparsity pattern.
    // VectorXd src(total_nodes);
    // VectorXd snk(total_nodes);
    // gc::SparseMat edge(total_nodes, total_nodes);
    // pc.applyWeights(weights_, &src, &snk, &edge);
    // cv::imshow("Final edges", visualizeEdges(edge));
    // cv::waitKey();
    
    // -- Generate the labels.
    if(!labels)
      return;

    ROS_ASSERT(seq->segmentations_.size() == seq->images_.size());
    *labels = VectorXi::Zero(total_nodes);
    for(size_t i = 0; i < seq->segmentations_.size(); ++i) {
      cv::Mat1b seg = seq->segmentations_[i];
      for(int y = 0; y < seg.rows; ++y) {
	for(int x = 0; x < seg.cols; ++x) {
	  if(seg(y, x) == 255) {
	    int idx = i * nodes_per_frame + y * seg.cols + x;
	    labels->coeffRef(idx) = 1;
	  }
	}
      }
    }
  }
  
  void VolumeSegmenter::cacheSequences(const std::vector<KinectSequence::Ptr>& sequences,
				       std::vector<gc::PotentialsCache::Ptr>* caches,
				       std::vector<gc::VecXiPtr>* labels) const
  {
    for(size_t i = 0; i < sequences.size(); ++i) {
      labels->push_back(gc::VecXiPtr(new gc::VecXi));
      caches->push_back(gc::PotentialsCache::Ptr(new gc::PotentialsCache));
      cacheSequence(sequences[i], caches->back(), labels->back());
    }
  }
  
  
}
