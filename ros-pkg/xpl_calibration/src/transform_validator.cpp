#include <xpl_calibration/transform_validator.h>
#include <pcl/common/transformation_from_correspondences.h>

using namespace std;
using namespace Eigen;
using namespace pcl;


#define VISUALIZE (getenv("VISUALIZE") ? atoi(getenv("VISUALIZE")) : 0)

TransformValidator::TransformValidator(double gamma) :
  gamma_(gamma)
{
}

Eigen::Affine3f TransformValidator::compute()
{
  // -- Try all candidates.  Choose the best.
  Eigen::Affine3f best_transform = Eigen::Affine3f::Identity();
  double best_loss = numeric_limits<double>::max();
  RGBDCloud transformed;
  RGBDCloud best_transformed;
  RGBDCloud::Ptr overlay(new RGBDCloud);
  visualization::CloudViewer vis("viewer");
  for(size_t i = 0; i < candidates_.size(); ++i) {
    transformed.clear();
    transformPointCloud(*tar_pcd_, transformed, candidates_[i]);
    double loss = computeLoss(*ref_pcd_, *ref_normals_, *ref_tree_, transformed);
    if(loss < best_loss) {
      best_loss = loss;
      best_transform = candidates_[i];
      best_transformed = transformed;
    }

    if(VISUALIZE) { 
      cout << "Validating " << i << " / " << candidates_.size() << ".  ";
      cout << "Loss = " << loss << ", best so far = " << best_loss << endl;

      overlay->clear();
      *overlay = *ref_pcd_;
      *overlay += best_transformed;
      vis.showCloud(overlay);
    }
  }

  
  // -- Run the best through ICP.
  fineTuneAlignment(*ref_pcd_, *ref_tree_, *ref_normals_, *tar_pcd_, &best_transform);
  return best_transform;
}

void TransformValidator::fineTuneAlignment(const RGBDCloud& ref,
					   search::KdTree<pcl::PointXYZRGB>& ref_tree,
					   const PointCloud<Normal>& ref_normals,
					   const RGBDCloud& tar,
					   Eigen::Affine3f* transform) const
{
  vector<int> indices(1);
  vector<float> distances(1);
  int iter = 0;
  RGBDCloud working;
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
  
double TransformValidator::computeLoss(const RGBDCloud& ref,
				       const PointCloud<Normal>& ref_normals,
				       pcl::search::KdTree<pcl::PointXYZRGB>& ref_tree,
				       const RGBDCloud& tar) const
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
    
