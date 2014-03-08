#ifndef CLUSTERER_H_
#define CLUSTERER_H_

class Dataset;
class GridClassifier;

// For retrospection, intersection_threshold = 0.7 and max_different_dspaces = 1 seemed
// pretty reasonable by eye.
bool similar(const Dataset& track0, const Dataset& track1,
             const GridClassifier& gc, double intersection_threshold,
             int max_different_dspaces);

#endif /* CLUSTERER_H_ */
