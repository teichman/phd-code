#ifndef CLUSTERER_H_
#define CLUSTERER_H_

#include <Eigen/Core>

class Dataset;
class GridClassifier;

// For retrospection, intersection_threshold = 0.7 and max_different_dspaces = 1 seemed
// pretty reasonable by eye.
bool similar(const Dataset& track0, const Dataset& track1,
             const GridClassifier& gc, double intersection_threshold,
             int max_different_dspaces);

Eigen::ArrayXi computeCellHistogram(const Dataset& track,
                                    size_t descriptor_id,
                                    const GridClassifier& gc);

Eigen::ArrayXf computeNormalizedCellHistogram(const Dataset& track,
                                              size_t descriptor_id,
                                              const GridClassifier& gc);

//! Returns true if all frames in the track are nearly the same in the context
//! of the GridClassifier gc.  All descriptor spaces are used.
//! The test value is the mean density of the grid cells that get activated by
//! at least one frame.
//! A totally static track will have a mean density of 1/num_grid_cells.
//! density_thresh of 0.5 and slack of 7 was reasonable once upon a time.
//! Probably you'll have to experiment for yourself though.
bool isStatic(const Dataset& track, const GridClassifier& gc,
              double density_thresh, size_t slack);


#endif /* CLUSTERER_H_ */
