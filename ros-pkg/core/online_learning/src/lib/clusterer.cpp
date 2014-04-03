#include <string>
#include <vector>

#include <Eigen/Core>
#include <ros/assert.h>
#include <online_learning/clusterer.h>
#include <online_learning/dataset.h>
#include <online_learning/grid_classifier.h>

using namespace std;
using Eigen::ArrayXf;
using Eigen::VectorXf;


Eigen::ArrayXf computeNormalizedCellHistogram(const Dataset& track,
                                              size_t descriptor_id,
                                              const GridClassifier& gc)
{
  ROS_ASSERT(gc.numResolutions() == 1);  // Only deal with a single grid resolution for now.  This could be extended later.
  vector<Grid*> grids = gc.grids_[0][descriptor_id];  // One grid per element of this descriptor space.
  ROS_ASSERT(!grids.empty());
  int num_cells_per_element = grids[0]->cells_.cols();  // Each element gets the same number of grid cells.
  int num_cells = grids.size() * num_cells_per_element;
  ArrayXf hist = ArrayXf::Zero(num_cells);
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

  hist /= hist.sum();
  return hist;
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
bool similar(const Dataset& track0, const Dataset& track1, const GridClassifier& gc, double intersection_threshold, int max_different_dspaces)
{
  ROS_ASSERT(track0.nameMappingsAreEqual(track1));
  const NameMapping& dmap = track0.nameMapping("dmap");

  vector<string> dspaces;
  // -- less-gravity
  dspaces.push_back("OrientedBoundingBoxSize.BoundingBoxSize:14462707047095316535");
  dspaces.push_back("CloudOrienter.Eigenvalues:12988381052486413110");
  dspaces.push_back("CloudOrienter.RelativeCurvature:1286978089874286107");
  dspaces.push_back("HSVHistogram.Hue:14694502210542588030");
  dspaces.push_back("HSVHistogram.Saturation:10273388249095023270");
  dspaces.push_back("HSVHistogram.Value:8985795375221662105");
//  dspaces.push_back("SimpleTrajectoryStatistics05.Speed:6152030001663489947");
  dspaces.push_back("SimpleTrajectoryStatistics10.Speed:9221731694321102499");
  dspaces.push_back("EdginessEstimator.Edginess:6941314615084190538");

  // -- old
  // dspaces.push_back("OrientedBoundingBoxSize.BoundingBoxSize:9048624352072648104");  // BoundingBoxSize from GravitationalCloudOrienter.
  // dspaces.push_back("CloudOrienter.Eigenvalues:12466250795116632929");
  // dspaces.push_back("CloudOrienter.RelativeCurvature:11309880616745749126");
  // dspaces.push_back("HSVHistogram.Hue:14694502210542588030");
  // dspaces.push_back("HSVHistogram.Saturation:10273388249095023270");
  // dspaces.push_back("HSVHistogram.Value:8985795375221662105");

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