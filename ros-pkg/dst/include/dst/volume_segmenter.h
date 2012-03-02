#ifndef VOLUME_SEGMENTER_H
#define VOLUME_SEGMENTER_H

#include <dst/segmentation_pipeline.h>
#include <graphcuts/maxflow_inference.h>

namespace dst
{

  class VolumeSegmenter
  {
  public:
    VolumeSegmenter(const Eigen::VectorXd& weights = Eigen::VectorXd());
    void segment(KinectSequence::ConstPtr seq,
		 std::vector<cv::Mat1b>* predictions) const;
    void cacheSequence(KinectSequence::ConstPtr seq,
		       graphcuts::PotentialsCache::Ptr pc,
		       graphcuts::VecXiPtr labels = graphcuts::VecXiPtr()) const;
    void cacheSequences(const std::vector<KinectSequence::Ptr>& sequences,
			std::vector<graphcuts::PotentialsCache::Ptr>* pcs,
			std::vector<graphcuts::VecXiPtr>* labels) const;
    
  protected:
    Eigen::VectorXd weights_;

    void addIntraFrameEdges(int offset,
			    FramePotentialsCache::ConstPtr cache,
			    graphcuts::PotentialsCache* pc) const;

    void addFlowEdges(int frame_id,
		      int epot_id,
		      int nodes_per_frame,
		      OpticalFlowNode::Output flow,
		      graphcuts::PotentialsCache* pc) const;

    void addDenseInterFrameEdges(int frame_id,
				 int epot_id,
				 int nodes_per_frame,
				 int total_nodes,
				 const KinectCloud& pcd,
				 KdTree::Ptr prev_kdtree,
				 graphcuts::PotentialsCache* pc) const;

  };

  cv::Mat1b visualizeEdges(const graphcuts::SparseMat& edges);

}

#endif // VOLUME_SEGMENTER_H
