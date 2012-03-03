#include <xpl_calibration/transform_validator.h>
#include <pcl/common/transformation_from_correspondences.h>

using namespace std;
using namespace Eigen;
using namespace pcl;
using namespace rgbd;

#define VISUALIZE (getenv("VISUALIZE") ? atoi(getenv("VISUALIZE")) : 0)

void TransformValidator::compute()
{

  const Candidates& candidates = *pull<CandidatesConstPtr>("Candidates");
  Cloud::ConstPtr cloud0 = pull<Cloud::ConstPtr>("Cloud0");
  Cloud::ConstPtr cloud1 = pull<Cloud::ConstPtr>("Cloud1");
  
  // -- Try all candidates.  Choose the best.
  Eigen::Affine3f best_transform = Eigen::Affine3f::Identity();
  double best_loss = numeric_limits<double>::max();
  Cloud transformed;
  Cloud best_transformed;
  Cloud::Ptr overlay(new Cloud);
  visualization::CloudViewer vis("viewer");
  for(size_t i = 0; i < candidates.size(); ++i) {
    transformed.clear();
    transformPointCloud(*cloud1, transformed, candidates[i]);
    double loss = computeLoss(*cloud0, *ref_normals_, *ref_tree_, transformed);
    if(loss < best_loss) {
      best_loss = loss;
      best_transform = candidates[i];
      best_transformed = transformed;
    }

    if(VISUALIZE) { 
      cout << "Validating " << i << " / " << candidates.size() << ".  ";
      cout << "Loss = " << loss << ", best so far = " << best_loss << endl;

      overlay->clear();
      *overlay = *cloud0;
      *overlay += best_transformed;
      vis.showCloud(overlay);
    }
  }

  
  // -- Run the best through ICP.
  fineTuneAlignment(*cloud0, *ref_tree_, *ref_normals_, *cloud1, &best_transform);
  return best_transform;
}

void TransformValidator::fineTuneAlignment(const Cloud& ref,
					   search::KdTree<pcl::PointXYZRGB>& ref_tree,
					   const PointCloud<Normal>& ref_normals,
					   const Cloud& tar,
					   Eigen::Affine3f* transform) const
{
  vector<int> indices(1);
  vector<float> distances(1);
  int iter = 0;
  Cloud working;
  transformPointCloud(tar, working, *transform);
  
  while(true) {
    cout << "Loss: " << computeLoss(ref, ref_normals, ref_tree, working) << endl;
    
    TransformationFromCorrespondences tfc;
    for(size_t i = 0; i < working.size(); ++i) { 
      indices.clear();
      distances.clear();
      ref_tree.nearestKSearch(working[i], 1, indices, distances);
      if(indices.empty() || distances[0] > 0.03)
	continue;
      
      tfc.add(working[i].getVector3fMap(), ref[indices[0]].getVector3fMap());
    }

    Affine3f tmptrans = tfc.getTransformation();
    pcl::transformPointCloud(working, working, tmptrans);
    *transform = tmptrans * (*transform);
    
    double delta_transform = (tmptrans.matrix() - Matrix4f::Identity()).norm();
    cout << "--------------------" << endl;
    cout << tmptrans.matrix() << endl;
    cout << "delta_transform: " << delta_transform << endl;
    if(delta_transform < 0.001)
      break;
    if(iter > 100) // TODO: Parameterize.
      break;
    ++iter;
  }
}
  
double TransformValidator::computeLoss(const Cloud& ref,
				       const PointCloud<Normal>& ref_normals,
				       pcl::search::KdTree<pcl::PointXYZRGB>& ref_tree,
				       const Cloud& tar) const
{
  double score = 0;
  double max_term = 0.1;
  int skip = 10;
  
  vector<int> indices;
  vector<float> distances;
  for(size_t i = 0; i < tar.size(); ++i) {
    if(isnan(tar[i].x))
      continue;

    if(rand() % skip != 0)
      continue;
    
    indices.clear();
    distances.clear();
    ref_tree.nearestKSearch(tar[i], 1, indices, distances);
    if(indices.empty())
      score += max_term;

    int idx = indices[0];
    Vector3f normal = ref_normals[idx].getNormalVector3fMap();
    double ptpdist = fabs(normal.dot(tar[i].getVector3fMap() - ref[idx].getVector3fMap()));
    Vector3f tc;
    tc(0) = (double)tar[i].r / 255.0;
    tc(1) = (double)tar[i].b / 255.0;
    tc(2) = (double)tar[i].g / 255.0;
    Vector3f rc;
    rc(0) = (double)ref[idx].r / 255.0;
    rc(1) = (double)ref[idx].b / 255.0;
    rc(2) = (double)ref[idx].g / 255.0;
    double cdist = (tc - rc).norm();

    score += min(max_term, ptpdist + gamma_ * cdist);
  }

  return score;
}
    
