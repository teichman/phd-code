#include <xpl_calibration/transform_validator.h>
#include <pcl/common/transformation_from_correspondences.h>

using namespace std;
using namespace Eigen;
using namespace pcl;
using namespace rgbd;

void TransformValidator::compute()
{
  best_transform_ = Affine3f::Identity();
  best_loss_ = numeric_limits<double>::max();
  
  const Candidates& candidates = *pull<const Candidates*>("Candidates");
  Cloud::ConstPtr cloud0 = pull<Cloud::ConstPtr>("Cloud0");
  Cloud::ConstPtr cloud1 = pull<Cloud::ConstPtr>("Cloud1");
  KdTree::Ptr tree0 = pull<KdTree::Ptr>("KdTree0");
  
  // -- Try all candidates.  Choose the best.
  Cloud transformed;
  Cloud best_transformed;
  Cloud::Ptr overlay(new Cloud);
  for(size_t i = 0; i < candidates.size(); ++i) {
    transformed.clear();
    transformPointCloud(*cloud1, transformed, candidates[i]);
    double loss = computeLoss(*cloud0, *tree0, transformed);
    if(loss < best_loss_) {
      best_loss_ = loss;
      best_transform_ = candidates[i];
      best_transformed = transformed;
    }
  }

  // -- Run the best through ICP.
  fineTuneAlignment(*cloud0, *tree0, *cloud1, &best_transform_);
  push<const Affine3f*>("BestTransform", &best_transform_);
}

void TransformValidator::fineTuneAlignment(const Cloud& cloud0,
                                           KdTree& tree0,
                                           const Cloud& cloud1,
                                           Eigen::Affine3f* transform) const
{
  vector<int> indices(1);
  vector<float> distances(1);
  int iter = 0;
  Cloud working;
  transformPointCloud(cloud1, working, *transform);
  
  while(true) {
    HighResTimer hrt("Computing loss");
    hrt.start();
    cout << "Loss: " << computeLoss(cloud0, tree0, working) << endl;
    hrt.stop();
    cout << hrt.report() << endl;

    hrt.reset("Getting transform");
    hrt.start();
    TransformationFromCorrespondences tfc;
    for(size_t i = 0; i < working.size(); ++i) {
      if(i % param<int>("Skip") != 0)
        continue;
      
      indices.clear();
      distances.clear();
      tree0.nearestKSearch(working[i], 1, indices, distances);
      if(indices.empty() || distances[0] > 0.03)
        continue;
      
      tfc.add(working[i].getVector3fMap(), cloud0[indices[0]].getVector3fMap());
    }

    Affine3f tmptrans = tfc.getTransformation();
    pcl::transformPointCloud(working, working, tmptrans);
    *transform = tmptrans * (*transform);
    hrt.stop();
    cout << hrt.report() << endl;
    
    double delta_transform = (tmptrans.matrix() - Matrix4f::Identity()).norm();
    cout << "--------------------" << endl;
    cout << tmptrans.matrix() << endl;
    cout << "delta_transform: " << delta_transform << endl;
    if(delta_transform < 0.001)
      break;
    if(iter > param<int>("MaxTuningIters"))
      break;
    ++iter;
  }
}
  
double TransformValidator::computeLoss(const Cloud& cloud0,
                                       KdTree& tree0,
                                       const Cloud& cloud1) const
{
  double score = 0;
  double max_term = 0.1;
  int skip = 10;
  
  vector<int> indices;
  vector<float> distances;
  for(size_t i = 0; i < cloud1.size(); ++i) {
    if(isnan(cloud1[i].x))
      continue;

    if(rand() % skip != 0)
      continue;
    
    indices.clear();
    distances.clear();
    tree0.nearestKSearch(cloud1[i], 1, indices, distances);
    if(indices.empty())
      score += max_term;

    int idx = indices[0];
    //Vector3f normal = cloud0_normals[idx].getNormalVector3fMap();
    //double ptpdist = fabs(normal.dot(cloud1[i].getVector3fMap() - cloud0[idx].getVector3fMap()));
    double dist = pcl::euclideanDistance(cloud1[i], cloud0[idx]);
    Vector3f tc;
    tc(0) = (double)cloud1[i].r / 255.0;
    tc(1) = (double)cloud1[i].b / 255.0;
    tc(2) = (double)cloud1[i].g / 255.0;
    Vector3f rc;
    rc(0) = (double)cloud0[idx].r / 255.0;
    rc(1) = (double)cloud0[idx].b / 255.0;
    rc(2) = (double)cloud0[idx].g / 255.0;
    double cdist = (tc - rc).norm();

    score += min(max_term, dist + param<double>("Gamma") * cdist);
  }

  return score;
}

void TransformValidator::debug() const
{
  const Candidates& candidates = *pull<const Candidates*>("Candidates");
  ofstream f((debugBasePath() + ".txt").c_str(), std::ios::out);
  f << "Checked " << candidates.size() << " transforms." << endl;
  f << "Best loss: " << best_loss_ << endl;
  f << "Best transform: " << endl << best_transform_.matrix() << endl;
  f.close();
  
  // overlay->clear();
  // *overlay = *cloud0;
  // *overlay += best_transformed;
  // vis.showCloud(overlay);
}
