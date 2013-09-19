#include <online_learning/classifier.h>

using namespace std;
using namespace Eigen;

Label Classifier::classifyTrack(const Dataset& track) const
{  
  Label prediction;
  prediction.setZero(nameMapping("cmap").size());
  for(size_t i = 0; i < track.instances_.size(); ++i)
    prediction += classify(track.instances_[i]) - prior();

  prediction /= (double)track.instances_.size();
  prediction += prior();
  return prediction;
}

void Trainer::train(TrackDataset::ConstPtr dataset)
{
  vector<TrackDataset::ConstPtr> datasets;
  datasets.push_back(dataset);
  vector<Indices> indices;
  indices.push_back(Indices::All(datasets[0]->size()));
  train(datasets, indices);
}

void Trainer::train(const Dataset& dataset)
{
  ROS_ASSERT(nameMappingsAreEqual(dataset));
  // TODO: Check that classifier has the same name mappings as the trainer.
  
  vector<int> index(dataset.size());
  for(size_t i = 0; i < index.size(); ++i)
    index[i] = i;
  random_shuffle(index.begin(), index.end());

  for(size_t i = 0; i < dataset.size(); ++i)
    train(dataset.instances_[index[i]]);
}
  
void Trainer::train(const TrackDataset& dataset)
{
  ROS_ASSERT(nameMappingsAreEqual(dataset));
  ROS_WARN("Not using minibatch");
  // TODO: Check that classifier has the same name mappings as the trainer.
  
  vector<Index> index(dataset.totalInstances());
  int idx = 0;
  for(size_t i = 0; i < dataset.size(); ++i)
    for(size_t j = 0; j < dataset[i].size(); ++j, ++idx)
      index[idx] = Index(-1, i, j);
  random_shuffle(index.begin(), index.end());

  for(size_t i = 0; i < index.size(); ++i)
    train(dataset[index[i].track_][index[i].frame_]);
}

void Trainer::trainSerial(const std::vector<TrackDataset::ConstPtr>& datasets,
			  std::vector<Indices> indices)
{
  ROS_ASSERT(indices.size() == datasets.size());
  for(size_t i = 0; i < datasets.size(); ++i)
    ROS_ASSERT(datasets[i]);
  
  // TODO: Check that classifier has the same name mappings as the trainer.

  // -- Get total number of instances.
  int total_num_instances = 0;
  for(size_t i = 0; i < indices.size(); ++i) {
    ROS_ASSERT(nameMappingsAreEqual(*datasets[i]));
    for(size_t j = 0; j < indices[i].size(); ++j) {
      int trid = indices[i][j];
      total_num_instances += datasets[i]->tracks_[trid]->size();
    }
  }
  cout << "Training on " << total_num_instances << " instances in " << datasets.size() << " datasets." << endl;

  // -- Build the index.
  vector<Index> index(total_num_instances);
  int idx = 0;
  for(size_t i = 0; i < indices.size(); ++i) {
    for(size_t j = 0; j < indices[i].size(); ++j) {
      int trid = indices[i][j];
      ROS_ASSERT(trid >= 0 && trid < (int)datasets[i]->tracks_.size());
      for(size_t k = 0; k < datasets[i]->tracks_[trid]->size(); ++k, ++idx)
	index[idx] = Index(i, trid, k);
    }
  }
  ROS_ASSERT(idx == total_num_instances);

  // -- Train in a random order.
  random_shuffle(index.begin(), index.end());
  for(size_t i = 0; i < index.size(); ++i) {
    if((int)i % (index.size() / 10) == 0)
      ROS_DEBUG_STREAM("Completed training on " << i << " / " << index.size() << " instances with stochastic logistic regression." << flush);

    ROS_ASSERT(index[i].dataset_ >= 0 && index[i].dataset_ < (int)datasets.size());
    train((*datasets[index[i].dataset_])[index[i].track_][index[i].frame_]);
  }
}

void Trainer::train(const std::vector<TrackDataset::ConstPtr>& datasets)
{
  // -- Construct full indices for all datasets.
  vector<Indices> indices;
  for(size_t i = 0; i < datasets.size(); ++i)
    indices.push_back(Indices::All(datasets[i]->size()));
  
  trainSerial(datasets, indices);
}

/************************************************************
 * Indices
 ************************************************************/

Indices Indices::All(size_t num)
{
  Indices ind;
  ind.resize(num);
  for(size_t i = 0; i < ind.size(); ++i)
    ind[i] = i;

  return ind;
}
