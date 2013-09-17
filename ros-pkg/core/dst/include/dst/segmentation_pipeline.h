#ifndef DST_PIPELINE_H
#define DST_PIPELINE_H

#include <Eigen/Eigen>
#include <opencv2/core/core.hpp>
#include <pcl/filters/filter.h>
#include <pipeline2/pipeline2.h>
#include <timer/timer.h>
#include <dst/compute_nodes.h>
#include <dst/potentials_cache.h>
#include <dst/kinect_sequence.h>

namespace dst
{
  
  //! Contains the pipeline2 for computing node and edge potentials,
  //! then running maxflow to get a segmentation.
  class SegmentationPipeline
  {
  public:
    typedef boost::shared_ptr<SegmentationPipeline> Ptr;

    pipeline2::Pipeline2 pipeline_;
    bool verbose_;
    SceneAlignmentNode* scene_alignment_node_;
    BoundaryMaskNode* boundary_mask_node_;
    
    SegmentationPipeline(int num_threads);
    virtual ~SegmentationPipeline() {}
    //! Returns a 1-channel uchar image.  0 = BG, 255 = FG.
    //! Only re-allocates the graph if necessary.
    //! Pass cv::Mat1b() to prev_seg if none.
    //! Pass cv::Mat3b() to prev_image if none.
    //! img_seg and pcd_seg come out.  img_seg must be the right size.
    //! pcd_seg will be filled.
    void run(cv::Mat1b seed,
             cv::Mat3b image,
             KinectCloud::ConstPtr cloud,
             cv::Mat3b prev_image,
             cv::Mat1b prev_seg,
             KinectCloud::ConstPtr prev_cloud,
             cv::Mat1b img_seg,
             KinectCloud::Ptr pcd_seg);

    //! Clears any accumulated data, e.g. color histogram models.
    void reset();
    void setDebug(bool debug);
    bool getDebug() const;
    void toggleDebug();
    std::string getGraphviz() const;
    Eigen::VectorXd getNodeWeights() const;
    Eigen::VectorXd getEdgeWeights() const;
    std::string weightsStatus() const;
    //! [edge^T node^T]
    void setWeights(const Eigen::VectorXd& weights, bool verbose = false);
    void setWeightsWithClipping(Eigen::VectorXd weights, bool verbose = false);
    //! [edge^T node^T]
    Eigen::VectorXd getWeights() const;
    PotentialsCache::Ptr cacheUnweightedPotentialsWithOracle(KinectSequence::Ptr seq);
    FramePotentialsCache::Ptr getFrameCache() const;
    //! Runs graph cuts on the cached potentials, adding a Hamming loss
    //! node potential that graph cuts will solve for the label of the
    //! most violated constraint in a structural SVM.
    cv::Mat1b findMostViolating(const FramePotentialsCache& fc,
                                cv::Mat1b labels,
                                bool hamming = true);
    cv::Mat3b getZBuffer(const KinectCloud& cloud,
                         int spread,
                         float min_range,
                         float max_range) const;
    cv::Mat3b getSurfNorm(const KinectCloud& cloud) const;
    void generateSegmentationFromGraph(Graph3d& graph,
                                       cv::Mat1i index,
                                       cv::Mat1b img_seg,
                                       KinectCloud::ConstPtr cloud =
                                       KinectCloud::ConstPtr((KinectCloud*)NULL),
                                       KinectCloud::Ptr pcd_seg =
                                       KinectCloud::Ptr((KinectCloud*)NULL)) const;

    //! Make a new SegmentationPipeline of this type.
    virtual SegmentationPipeline::Ptr mitosis(int num_threads) const;
    void getNameMappings(NameMapping* epot_names, NameMapping* npot_names) const;
    
  protected:
    Graph3dPtr graph_;
    EntryPoint<cv::Mat3b>* image_ep_;
    EntryPoint<cv::Mat3b>* previous_image_ep_;
    EntryPoint<cv::Mat1b>* seed_ep_;
    EntryPoint<cv::Mat1b>* previous_segmentation_ep_;
    EntryPoint<KinectCloud::ConstPtr>* cloud_ep_;
    EntryPoint<KinectCloud::ConstPtr>* previous_cloud_ep_;
    EntryPoint<Graph3dPtr>* graph_ep_;
    EdgePotentialAggregator* edge_aggregator_;
    NodePotentialAggregator* node_aggregator_;
    DepthProjector* depth_projector_;
    OrganizedSurfaceNormalNode* normals_node_;
    int num_threads_;
  };

} // namespace dst


#endif // DST_PIPELINE_H
