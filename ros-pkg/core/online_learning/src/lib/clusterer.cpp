#include <string>
#include <vector>

#include <Eigen/Core>
#include <ros/assert.h>
#include <online_learning/clusterer.h>
#include <online_learning/dataset.h>
#include <online_learning/grid_classifier.h>

using namespace std;
using Eigen::ArrayXf;
using Eigen::ArrayXi;
using Eigen::VectorXf;


Eigen::ArrayXf computeNormalizedCellHistogram(const Dataset& track,
                                              size_t descriptor_id,
                                              const GridClassifier& gc)
{
  ArrayXf hist = computeCellHistogram(track, descriptor_id, gc).cast<float>();
  return hist / hist.sum();
}

float histogramIntersection(const Eigen::ArrayXf& hist0, const Eigen::ArrayXf& hist1)
{
  ROS_ASSERT(hist0.rows() == hist1.rows());

  float val = 0;
  for(int i = 0; i < hist0.rows(); ++i)
    val += min(hist0.coeffRef(i), hist1.coeffRef(i));
  return val;
}


//! annotation and inducted have descriptors computed.
// TODO: Anything that references particular descriptors does not belong in the online_learning package.
// Perhaps there should be a class for computing similarities which can be loaded with the descriptor spaces
// it is to care about.
bool similar(const Dataset& track0, const Dataset& track1, const GridClassifier& gc, double intersection_threshold, int max_different_dspaces)
{
  ROS_ASSERT(track0.nameMappingsAreEqual(track1));
  const NameMapping& dmap = track0.nameMapping("dmap");

  if(track0.empty() || track1.empty())
    return false;

  vector<string> dspaces;
  // -- less-gravity with Blob::Ptr
  // dspaces.push_back("OrientedBoundingBoxSize.BoundingBoxSize:14462707047095316535");
  // dspaces.push_back("CloudOrienter.Eigenvalues:12988381052486413110");
  // dspaces.push_back("CloudOrienter.RelativeCurvature:1286978089874286107");
  // dspaces.push_back("HSVHistogram.Hue:14694502210542588030");
  // dspaces.push_back("HSVHistogram.Saturation:10273388249095023270");
  // dspaces.push_back("HSVHistogram.Value:8985795375221662105");
  // dspaces.push_back("SimpleTrajectoryStatistics10.Speed:9221731694321102499");
  // dspaces.push_back("EdginessEstimator.Edginess:6941314615084190538");

  // -- less-gravity with Blob::ConstPtr
  //    TODO: online_learning should have a SimilarityTester class which applies the logic
  //    below with whatever descriptor spaces a user specifies.
  dspaces.push_back("OrientedBoundingBoxSize.BoundingBoxSize:12128581193421816702");
  dspaces.push_back("CloudOrienter.Eigenvalues:6198792747096408541");
  dspaces.push_back("CloudOrienter.RelativeCurvature:15496457168725876226");
  dspaces.push_back("HSVHistogram.Hue:14225771044352461861");
  dspaces.push_back("HSVHistogram.Saturation:7223821915271586925");
  dspaces.push_back("HSVHistogram.Value:15046389458041709312");
  dspaces.push_back("SimpleTrajectoryStatistics10.Speed:10496021830416604330");
  dspaces.push_back("EdginessEstimator.Edginess:12335766797991983889");

  int num_different_dspaces = 0;
  for(size_t i = 0; i < dspaces.size(); ++i) {
    if(!dmap.hasName(dspaces[i])) {
      cout << dmap << endl;
      ROS_ASSERT(dmap.hasName(dspaces[i]));
    }

    size_t id = dmap.toId(dspaces[i]);
    ArrayXf hist0 = computeNormalizedCellHistogram(track0, id, gc);
    ArrayXf hist1 = computeNormalizedCellHistogram(track1, id, gc);

    if(histogramIntersection(hist0, hist1) < intersection_threshold) {
      ++num_different_dspaces;
      if(num_different_dspaces > max_different_dspaces)
        return false;
    }
  }
  return true;
}

bool similar(const Dataset& annotation, const Dataset& inducted)
{
  const NameMapping& dmap = annotation.nameMapping("dmap");

  string name = "OrientedBoundingBoxSize.BoundingBoxSize:14462707047095316535";  // BoundingBoxSize from CloudOrienter.
  //string name = "OrientedBoundingBoxSize.BoundingBoxSize:9048624352072648104";  // BoundingBoxSize from GravitationalCloudOrienter.
  if(!dmap.hasName(name)) {
    cout << dmap << endl;
    ROS_ASSERT(dmap.hasName(name));
  }
  size_t id = dmap.toId(name);

  double min_dist = numeric_limits<double>::max();
  int num_samples = 10;
  for(int i = 0; i < num_samples; ++i) {
    VectorXf* ann = annotation[rand() % annotation.size()].descriptors_[id];
    ROS_ASSERT(ann);
    ROS_ASSERT(ann->rows() == 3);
    for(int j = 0; j < num_samples; ++j) {
      VectorXf* ind = inducted[rand() % inducted.size()].descriptors_[id];
      ROS_ASSERT(ind);
      ROS_ASSERT(ind->rows() == 3);

      double max_pct_change = 0;
      for(int k = 0; k < ind->rows(); ++k) {
        // double pct_change = max(fabs(ann->coeffRef(k) - ind->coeffRef(k)) / ann->coeffRef(k),
        //                         fabs(ann->coeffRef(k) - ind->coeffRef(k)) / ind->coeffRef(k));
        if(ind->coeffRef(k) == 0)
          continue;
        double pct_change = fabs(ann->coeffRef(k) - ind->coeffRef(k)) / ind->coeffRef(k);
        max_pct_change = max(max_pct_change, pct_change);
      }

      min_dist = min(min_dist, max_pct_change);
    }
  }
  return (min_dist < 0.2);
}

Eigen::ArrayXi computeCellHistogram(const Dataset& track,
                                    size_t descriptor_id,
                                    const GridClassifier& gc)
{
  ROS_ASSERT(gc.numResolutions() == 1);  // Only deal with a single grid resolution for now.  This could be extended later.
  vector<Grid*> grids = gc.grids_[0][descriptor_id];  // One grid per element of this descriptor space.
  ROS_ASSERT(!grids.empty());
  int num_cells_per_element = grids[0]->cells_.cols();  // Each element gets the same number of grid cells.
  int num_cells = grids.size() * num_cells_per_element;
  ArrayXi hist = ArrayXi::Zero(num_cells);
  for(size_t i = 0; i < grids.size(); ++i) {
    ROS_ASSERT(grids[i]);
    const Grid& grid = *grids[i];
    ROS_ASSERT(i == 0 || grid.cells_.cols() == grids[i-1]->cells_.cols());

    for(size_t j = 0; j < track.size(); ++j) {
      const Instance& frame = track[j];
      if(!frame[descriptor_id])
        continue;

      const VectorXf& descriptor = *frame[descriptor_id];
      size_t idx = grid.getCellIdx(descriptor.coeffRef(i));
      ROS_ASSERT(i * num_cells_per_element + idx < (size_t)hist.rows());
      ++hist(i * num_cells_per_element + idx);
    }
  }
  return hist;
}

bool isStatic(const Dataset& track, const GridClassifier& gc,
              double density_thresh, size_t slack)
{
  ROS_ASSERT(track.nameMapping("dmap") == gc.nameMapping("dmap"));
  const NameMapping& dmap = track.nameMapping("dmap");

  ArrayXi hist;
  size_t num = 0;
  for(size_t i = 0; i < dmap.size(); ++i) {
    hist = computeCellHistogram(track, i, gc);
    //cout << hist.transpose() << endl;
    double density = (double)(hist != 0).count() / hist.rows();
    if(density < 0 || density > 1) {
      ROS_WARN_STREAM("Unexpected density value: " << density);
      ROS_ASSERT(0);
    }
    //cout << density << endl;
    
    if(density > density_thresh) {
      ++num;
      if(num >= slack)
        return false;
    }
  }
  return true;
}
