#include <xpl_calibration/xpl_calibrator.h>
#include <pcl/common/transformation_from_correspondences.h>

using namespace std;
using namespace Eigen;
using namespace pcl;


#define VISUALIZE (getenv("VISUALIZE") ? atoi(getenv("VISUALIZE")) : 0)

std::ostream& operator<<(std::ostream& out, const Junction& junc)
{
  out << "Junction" << endl;
  out << "  Point 1: " << junc.pt1_.transpose() << endl;
  out << "  Point 2: " << junc.pt2_.transpose() << endl;
  out << "  Point 3: " << junc.pt3_.transpose() << endl;
  out << "  Normal 1: " << junc.normal1_.transpose() << endl;
  out << "  Normal 2: " << junc.normal2_.transpose() << endl;
  out << "  Crease direction: " << junc.creasedir_.transpose() << endl;
  out << "  Color 1: " << junc.color1_.transpose() << endl;
  out << "  Color 2: " << junc.color2_.transpose() << endl;
  out << "  Min: " << junc.min_ << endl;
  out << "  Max: " << junc.max_ << endl;
  return out;
}

void Junction::swap()
{
  Vector3f tmp = pt1_;
  pt1_ = pt2_;
  pt2_ = tmp;

  tmp = normal1_;
  normal1_ = normal2_;
  normal2_ = tmp;
  creasedir_ = -creasedir_;

  tmp = color1_;
  color1_ = color2_;
  color2_ = tmp;

  double min2 = -max_;
  double max2 = -min_;
  max_ = max2;
  min_ = min2;
}

XplCalibrator::XplCalibrator() :
  distance_thresh_(0.4),
  min_angle_(M_PI / 4.0),
  gamma_(0.1),
  granularity_(0.05)
{
}

void XplCalibrator::findJunctions(const RGBDCloud& pcd,
				  const PointCloud<Normal>& normals,
				  vector<Junction>* junctions) const
{
  // -- Find planes.
  // Choose a reasonable number of min inliers for whatever resolution.
  int min_pts = (double)(pcd.width * pcd.height) / 96.0;  
  PlaneFinder pf(min_pts, 20.0 * M_PI / 180.0, 0.2);
  vector<int> assignments;
  pf.compute(pcd, normals);
  
  // -- Find the intersections of all the planes.
  //    Only take those that are actually touching, i.e.
  //    when a few points on one plane are projected onto
  //    the other, there are near neighbors.
  set< pair<int, int> > adj;
  for(size_t i = 0; i < pcd.size(); ++i) {
    if(pf.assignments_[i] < 0)
      continue;
    int p1 = pf.assignments_[i];
    
    ImageRegionIterator iri(cv::Size(pcd.width, pcd.height), 10);
    for(iri.setCenter(i); !iri.done(); ++iri) {
      int p2 = pf.assignments_[iri.index()];
      if(p2 < 0 || p2 == p1)
	continue;
      
      double dist = pcl::euclideanDistance(pcd[iri.index()], pcd[i]);
      double angle = acos(fabs(pf.normals_[p1].dot(pf.normals_[p2])));
      if(dist < distance_thresh_ && angle > min_angle_)
	adj.insert(pair<int, int>(min(p1, p2), max(p1, p2)));
    }
  }
    
  // -- Define sets of three points for each pair of adjacent planes.
  set< pair<int, int> >::const_iterator it;
  for(it = adj.begin(); it != adj.end(); ++it) {
    int p1 = it->first;
    int p2 = it->second;
    Junction junc;
    junc.normal1_ = pf.normals_[p1];
    junc.normal2_ = pf.normals_[p2];
    junc.creasedir_ = pf.normals_[p1].cross(pf.normals_[p2]);
    junc.creasedir_.normalize();

    // Get image centroids, for visualization.
    junc.img_centroid1_ = pf.img_centroids_[p1];
    junc.img_centroid2_ = pf.img_centroids_[p2];
        
    // Get centroid of points on the crease direction.
    double c = 0;
    double num = 0;
    junc.min_ = numeric_limits<double>::max();
    junc.max_ = -numeric_limits<double>::max();
    for(size_t i = 0; i < pcd.size(); ++i) {
      if(pf.assignments_[i] == p1 || pf.assignments_[i] == p2) { 
	++num;
	double val = junc.creasedir_.dot(pcd[i].getVector3fMap());
	c += val;
	if(val < junc.min_)
	  junc.min_ = val;
	if(val > junc.max_)
	  junc.max_= val;
      }
    }
    c /= num;

    // Solve for pt3.  It should lie at the intersection of the two planes
    // and be at the centroid of the points on creasedir as computed above.
    Matrix3f m;
    m.row(0) = junc.normal1_;
    m.row(1) = junc.normal2_;
    m.row(2) = junc.creasedir_;
    Vector3f b = Vector3f::Zero();
    b(2) = c;
    num = 0;
    for(size_t i = 0; i < pcd.size(); ++i) {
      if(pf.assignments_[i] == p1) {
	++num;
	b(0) += junc.normal1_.dot(pcd[i].getVector3fMap());
      }
    }
    b(0) /= num;
    num = 0;
    for(size_t i = 0; i < pcd.size(); ++i) {
      if(pf.assignments_[i] == p2) {
	++num;
	b(1) += junc.normal2_.dot(pcd[i].getVector3fMap());
      }
    }
    b(1) /= num;

    junc.pt3_ = m.inverse() * b;

    // The other points are placed at one meter from pt3, on their respective planes.
    junc.pt1_ = junc.pt3_ + junc.normal1_;
    junc.pt2_ = junc.pt3_ + junc.normal2_;
    
    cout << "Adding junction: " << endl << junc << endl;
    junctions->push_back(junc);
  }
}

cv::Mat3b XplCalibrator::visualizeJunctions(const std::vector<Junction>& junctions,
					    cv::Mat3b img) const
{
  cv::Mat3b vis = img.clone();
  for(size_t i = 0; i < junctions.size(); ++i)
    cv::line(vis, junctions[i].img_centroid1_, junctions[i].img_centroid2_, cv::Scalar(255, 0, 0));

  return vis;
}


Eigen::Affine3f XplCalibrator::calibrate(RGBDSequence::ConstPtr refseq,
					 RGBDSequence::ConstPtr tarseq) const
{
  vector<Junction> ref_junctions;
  vector<Junction> tar_junctions;
  RGBDCloud::Ptr ref = refseq->pcds_[0];
  RGBDCloud::Ptr tar = tarseq->pcds_[0];

  HighResTimer hrt("KdTrees");
  hrt.start();
  search::KdTree<pcl::PointXYZRGB>::Ptr ref_tree(new search::KdTree<pcl::PointXYZRGB>);
  ref_tree->setInputCloud(ref);
  search::KdTree<pcl::PointXYZRGB>::Ptr tar_tree(new search::KdTree<pcl::PointXYZRGB>);
  tar_tree->setInputCloud(tar);
  hrt.stop();
  cout << hrt.report() << endl;

  hrt.reset("Normals");
  hrt.start();

  // pipeline2::Outlet<RGBDCloud::ConstPtr> pcd_otl(NULL);
  // pipeline2::Outlet<cv::Mat1b> mask_otl(NULL);
  // pcd_otl.push(ref);
  // mask_otl.push(cv::Mat1b(cv::Size(ref->width, ref->height), 255));

  // OrganizedSurfaceNormalNode osn(&pcd_otl, &mask_otl, 20);
  // osn._compute();
  // PointCloud<Normal>::ConstPtr ref_normals = osn.normals_otl_.pull();

  // osn._flush();
  // pcd_otl.push(tar);
  // osn._compute();
  // PointCloud<Normal>::ConstPtr tar_normals = osn.normals_otl_.pull();
  
  NormalEstimation<pcl::PointXYZRGB, Normal> ref_ne;
  ref_ne.setSearchMethod(ref_tree);
  ref_ne.setInputCloud(ref);
  ref_ne.setRadiusSearch(0.3);
  PointCloud<Normal>::Ptr ref_normals(new PointCloud<Normal>);
  ref_ne.compute(*ref_normals);
  
  NormalEstimation<pcl::PointXYZRGB, Normal> tar_ne;
  tar_ne.setSearchMethod(tar_tree);
  tar_ne.setInputCloud(tar);
  tar_ne.setRadiusSearch(0.3);
  PointCloud<Normal>::Ptr tar_normals(new PointCloud<Normal>);
  tar_ne.compute(*tar_normals);

  hrt.stop();
  cout << hrt.report() << endl;

  pcl::PointCloud<pcl::PointXYZRGBNormal> cn;
  pcl::concatenateFields(*ref, *ref_normals, cn);
  pcl::io::savePCDFileBinary("normals.pcd", cn);
  
  hrt.reset("findJunctions");
  hrt.start();
  findJunctions(*ref, *ref_normals, &ref_junctions);
  findJunctions(*tar, *tar_normals, &tar_junctions);
  hrt.stop();
  cout << hrt.report() << endl;

  if(VISUALIZE) {
    cv::imshow("ref junctions", visualizeJunctions(ref_junctions, refseq->imgs_[0]));
    cv::imshow("tar junctions", visualizeJunctions(tar_junctions, tarseq->imgs_[0]));
    cv::waitKey();
  }
  
  visualization::CloudViewer vis("viewer");
  double best_loss = numeric_limits<double>::max();
  Affine3f best_transform = Affine3f::Identity();
  RGBDCloud transformed = *ref;
  RGBDCloud::Ptr overlay(new RGBDCloud);
  for(size_t i = 0; i < ref_junctions.size(); ++i) {
    for(size_t j = 0; j < tar_junctions.size(); ++j) {
  // for(size_t i = 0; i < 1; ++i) {
  //   for(size_t j = 0; j < 1; ++j) {
      cout << "Trying " << i << " " << j << endl;
      
      for(int k = 0; k < 2; ++k) { 
	Junction& rj = ref_junctions[i];
	Junction tj = tar_junctions[j];
	if(k == 1)
	  tj.swap();

	if(VISUALIZE) {
	  cv::Mat3b refvis = refseq->imgs_[0].clone();
	  cv::line(refvis, rj.img_centroid1_, rj.img_centroid2_, cv::Scalar(255, 0, 0));
	  cv::imshow("ref junction", refvis);
	  
	  cv::Mat3b tarvis = tarseq->imgs_[0].clone();
	  cv::line(tarvis, tj.img_centroid1_, tj.img_centroid2_, cv::Scalar(255, 0, 0));
	  cv::imshow("tar junction", tarvis);
	  
	  cv::waitKey(20);
	}
      	
	Affine3f base_transform;
	bool valid = computeTransform(rj, tj, &base_transform);
	if(!valid) {
	  if(VISUALIZE)
	    cout << "Transform not valid.  Points probably did not line up.  Continuing." << endl;
	  continue;
	}
	
	RGBDCloud base_transformed;
	transformPointCloud(*tar, base_transformed, base_transform);

	// Get the min and max.
	double ref_min = numeric_limits<double>::max();
	double ref_max = -numeric_limits<double>::max();
	for(size_t l = 0; l < ref->size(); ++l) {
	  double val = rj.creasedir_.dot(ref->at(l).getVector3fMap());
	  if(val > ref_max)
	    ref_max = val;
	  if(val < ref_min)
	    ref_min = val;
	}
	double tar_min = numeric_limits<double>::max();
	double tar_max = -numeric_limits<double>::max();
	for(size_t l = 0; l < base_transformed.size(); ++l) {
	  double val = rj.creasedir_.dot(base_transformed[l].getVector3fMap());
	  if(val > tar_max)
	    tar_max = val;
	  if(val < tar_min)
	    tar_min = val;
	}

	// Search over possible translations.
	double lower_limit = ref_min - tar_max;
	double upper_limit = ref_max - tar_min;
	double range = upper_limit - lower_limit;
	for(double offset = lower_limit + 0.25 * range; offset <= upper_limit - 0.25 * range; offset += granularity_) {
	  Vector3f translation = offset * rj.creasedir_;
	  Affine3f transform = base_transform;
	  transform.translation() += translation;
	  transformPointCloud(*tar, transformed, transform);
	  //applyTranslation(base_transformed, translation, &transformed);
	  double loss = computeLoss(*ref, *ref_normals, *ref_tree, transformed);

	  if(loss < best_loss) {
	    best_loss = loss;
	    best_transform = transform;
	  }

	  if(VISUALIZE) { 
	    overlay->clear();
	    *overlay = *ref;
	    *overlay += transformed;
	    vis.showCloud(overlay);
	    vis.wasStopped(1);
	    cout << "Loss: " << loss << endl;
	    //cin.ignore();
	  }
	}	
      }
    }
  }

  // -- Visualize
  if(VISUALIZE) { 
    overlay->clear();
    *overlay = *ref;
    transformed.clear();
    transformPointCloud(*tar, transformed, best_transform);
    *overlay += transformed;
    vis.showCloud(overlay);
    cout << "Displaying best transform, before ICP final stage." << endl;
    cin.ignore();
  }
  
  // -- Run ICP to get final alignment.
  fineTuneAlignment(*ref, *ref_tree, *ref_normals, *tar, &best_transform);

  if(VISUALIZE) { 
    overlay->clear();
    *overlay = *ref;
    transformed.clear();
    transformPointCloud(*tar, transformed, best_transform);
    *overlay += transformed;
    vis.showCloud(overlay);
    cout << "Displaying best transform, after ICP final stage." << endl;
    cin.ignore();
  }
  
  return best_transform;
}

void XplCalibrator::fineTuneAlignment(const RGBDCloud& ref,
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
  
void XplCalibrator::applyTranslation(const RGBDCloud& src, const Vector3f& translation, RGBDCloud* dst) const
{
  ROS_ASSERT(src.size() == dst->size());
  for(size_t i = 0; i < src.size(); ++i)
    if(!isnan(src[i].x))
      dst->at(i).getVector3fMap() = src[i].getVector3fMap() + translation;
}

double XplCalibrator::computeLoss(const RGBDCloud& ref,
				  const PointCloud<Normal>& ref_normals,
				  pcl::search::KdTree<pcl::PointXYZRGB>& ref_tree,
				  const RGBDCloud& tar) const
{
  double score = 0;
  double max_term = 0.1;
  
  vector<int> indices;
  vector<float> distances;
  for(size_t i = 0; i < tar.size(); ++i) {
    if(isnan(tar[i].x))
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

// T * tar = ref
bool XplCalibrator::computeTransform(const Junction& ref, const Junction& tar, Eigen::Affine3f* transform) const
{
  TransformationFromCorrespondences tfc;
  tfc.add(tar.pt1_, ref.pt1_);
  tfc.add(tar.pt2_, ref.pt2_);
  tfc.add(tar.pt3_, ref.pt3_);
  *transform = tfc.getTransformation();

  if((*transform * tar.pt1_ - ref.pt1_).norm() > 0.1 ||
     (*transform * tar.pt2_ - ref.pt2_).norm() > 0.1 ||
     (*transform * tar.pt3_ - ref.pt3_).norm() > 0.1)
    return false;
  else
    return true;
}
    
