#include <gtest/gtest.h>
#include <online_learning/dataset.h>
#include <online_learning/synthetic_data_generator.h>
#include <online_learning/training_buffer.h>
#include <bag_of_tricks/high_res_timer.h>

using namespace std;
using namespace Eigen;

TEST(Dataset, HashSynthetic)
{
  int num_descriptors = 3;
  SyntheticDataGenerator sdg(getDefaultDimensionality(num_descriptors), 10, 1, 0.2, defaultClassMap(), getStubDescriptorMap(num_descriptors));
  Dataset::Ptr dataset = sdg.sampleDataset(1000);

  dataset->save("test_dataset");
  Dataset dataset2;
  dataset2.load("test_dataset");
  EXPECT_EQ(dataset->hash(), dataset2.hash());

  set<double> hashes;
  for(size_t i = 0; i < dataset->size(); ++i)
    hashes.insert((*dataset)[i].hash());
  cout << "num collisions: " << dataset->size() - hashes.size() << endl;
}

TEST(Dataset, HashReal)
{
  if(getenv("GTEST_HASH_DATASET_PATH")) {
    string path = getenv("GTEST_HASH_DATASET_PATH");
    TrackDataset td;
    td.load(path);

    set<double> hashes_track;
    HighResTimer hrt_track("Track hashing");
    for(size_t i = 0; i < td.size(); ++i) {
      hrt_track.start();
      double val = td[i].hash();
      hrt_track.stop();
      hashes_track.insert(val);
    }
    cout << "Num track hash collisions: " << td.size() - hashes_track.size() << endl;
    cout << "Mean track hash time: " << hrt_track.getMilliseconds() / td.size() << " ms." << endl;

    for(size_t i = 0; i < td.size(); ++i)
      EXPECT_TRUE(hashes_track.count(td[i].hash()));
    
    set<double> hashes_frame;
    HighResTimer hrt_frame("Frame hashing");
    for(size_t i = 0; i < td.size(); ++i) {
      for(size_t j = 0; j < td[i].size(); ++j) {
        hrt_frame.start();
        double val = td[i][j].hash();
        hrt_frame.stop();
        hashes_frame.insert(val);
      }
    }
    cout << "Num frame hash collisions: " << td.totalInstances() - hashes_frame.size() << endl;
    cout << "Mean frame hash time: " << hrt_frame.getMilliseconds() / td.totalInstances() << " ms." << endl;

    for(size_t i = 0; i < td.size(); ++i)
      for(size_t j = 0; j < td[i].size(); ++j)
        EXPECT_TRUE(hashes_frame.count(td[i][j].hash()));
  }
}

TEST(Dataset, Serialization)
{
  int num_descriptors = 3;
  SyntheticDataGenerator sdg(getDefaultDimensionality(num_descriptors), 10, 1, 0.2, defaultClassMap(), getStubDescriptorMap(num_descriptors));
  Dataset::Ptr dataset = sdg.sampleDataset(1000);
  cout << dataset->status() << endl;
  cout << "Some random instances: " << endl;
  srand(0);
  int num_random_instances = 3;
  for(int i = 0; i < num_random_instances; ++i) {
    int idx = rand() % dataset->instances_.size();
    cout << dataset->instances_[idx].status("  ");
  }
  
  Dataset copy = *dataset;
  EXPECT_TRUE(copy == *dataset);
  cout << "Some random instances from the copy: " << endl;
  srand(0);
  for(int i = 0; i < num_random_instances; ++i) {
    int idx = rand() % copy.instances_.size();
    cout << copy.instances_[idx].status("  ");
  }

  dataset->save("test_dataset");
  Dataset serialized;
  serialized.load("test_dataset");
  EXPECT_TRUE(serialized == *dataset);

  cout << "Some random instances from the serialized version: " << endl;
  srand(0);
  for(int i = 0; i < num_random_instances; ++i) {
    int idx = rand() % serialized.instances_.size();
    cout << serialized.instances_[idx].status("  ");
  }

  // Testing deallocation of Instances.
  serialized = copy;
}

TEST(TrackDataset, NameMapping)
{
  int num_descriptors = 3;
  SyntheticDataGenerator sdg(getDefaultDimensionality(num_descriptors), 10, 1, 0.1, defaultClassMap(), getStubDescriptorMap(num_descriptors));
  TrackDataset::Ptr td = sdg.sampleTrackDataset(5, 1000);
  cout << td->status() << endl;

  TrackDataset::Ptr copy = td->clone();
  NameMapping cmap;
  cmap.addName("pedestrian");
  cmap.addName("bicyclist");
  cmap.addName("car");
  NameMapping dmap;
  for(int i = td->nameMapping("dmap").size() - 1; i >= 0; --i)
    dmap.addName(td->nameMapping("dmap").toName(i));
  td->applyNameMapping("cmap", cmap);
  td->applyNameMapping("dmap", dmap);
  EXPECT_TRUE(*copy != *td);
  EXPECT_TRUE(!td->tracks_[0]->nameMappingsAreEqual(*copy));
  EXPECT_TRUE(!(*copy)[0].nameMappingsAreEqual(*td));
  EXPECT_TRUE((*copy)[0].nameMappingsAreEqual(*copy));
  EXPECT_TRUE((*td)[0].nameMappingsAreEqual(*td));
  td->applyNameMappings(*copy);
  EXPECT_TRUE(td->tracks_[0]->nameMappingsAreEqual(*copy));
  EXPECT_TRUE((*copy)[0].nameMappingsAreEqual(*td));
  EXPECT_TRUE(*copy == *td);
  EXPECT_TRUE((*copy)[0].nameMappingsAreEqual(*copy));
  EXPECT_TRUE((*td)[0].nameMappingsAreEqual(*td));

  TrackDataset serialized;
  td->save("track_dataset_test");
  serialized.load("track_dataset_test");
  EXPECT_TRUE(*td == serialized);
}

TEST(Dataset, MissingDescriptors)
{
  int num_descriptors = 4;
  SyntheticDataGenerator sdg(getDefaultDimensionality(num_descriptors), 10, 1, 0.1, defaultClassMap(), getStubDescriptorMap(num_descriptors));
  Dataset::Ptr dataset = sdg.sampleDataset(100);
  TrackDataset td;
  td.tracks_.push_back(dataset);
  DescriptorDimensionality dim;
  dim = td.inferDescriptorDimensionality();
  EXPECT_TRUE(dim.total() == 10);
  EXPECT_TRUE(dim.size() == 4);
  delete dataset->instances_[0][0];
  dataset->instances_[0][0] = NULL;
  delete dataset->instances_[1][3];
  dataset->instances_[1][3] = NULL;
  delete dataset->instances_[2][1];
  dataset->instances_[2][1] = NULL;
  dim = td.inferDescriptorDimensionality();
  EXPECT_TRUE(dim.total() == 10);
  EXPECT_TRUE(dim.size() == 4);
  cout << "Num descriptors: " << dim.size() << endl;
}

TEST(Label, id)
{
  Label label = VectorXf::Zero(3);
  EXPECT_TRUE(label.id() == -2);
  label(1) = 1;
  EXPECT_TRUE(label.id() == 1);
  label(1) = -1;
  EXPECT_TRUE(label.id() == -1);
  label(2) = 0;
  EXPECT_TRUE(label.id() == -1);
}

TEST(TrainingBuffer, MaskedMerge)
{
  if(getenv("DATASET_PATH")) {
    string dataset_path = getenv("DATASET_PATH");
    vector<string> paths;
    paths.push_back(dataset_path + "/workspace/datasets/hand_labeled/raw/background/null20-11-16-2009_21-12-04.td");
    paths.push_back(dataset_path + "/workspace/datasets/hand_labeled/raw/background/null06-11-16-2009_20-30-22.td");
    paths.push_back(dataset_path + "/workspace/datasets/hand_labeled/raw/background/null10-11-16-2009_20-40-17.td");
    paths.push_back(dataset_path + "/workspace/datasets/hand_labeled/raw/natural/CampusLoop3-10-03-2009_15-32-37-FirstMinute.td");
    paths.push_back(dataset_path + "/workspace/datasets/hand_labeled/raw/natural/intersection-04-07-2010_09-05-08.td");
    paths.push_back(dataset_path + "/workspace/datasets/hand_labeled/raw/natural/page_mill_and_hansen02-11-17-2009_19-56-57.td");
    paths.push_back(dataset_path + "/workspace/datasets/hand_labeled/raw/natural/pass2-10-16-2009_15-00-46.td");
    paths.push_back(dataset_path + "/workspace/datasets/hand_labeled/raw/natural/white_plaza02-11-17-2009_17-35-42.td");
    
    cout << "Initializing." << endl;
    TrackDataset::Ptr init(new TrackDataset);
    init->load(paths[0]);
    int max_frames = -1;
    if(getenv("MAX_FRAMES"))
      max_frames = atoi(getenv("MAX_FRAMES"));

    cout << "Using max_frames_ = " << max_frames << endl;
    TrainingBuffer tb;
    tb.max_frames_ = max_frames;
    tb.applyNameMappings(*init);
    tb.init(3000);
    init.reset();

    do {
      for(size_t i = 0; i < paths.size(); ++i) {
	TrackDataset dataset;
	cout << "Loading " << paths[i] << endl;
	dataset.load(paths[i]);

	if(getenv("EXCLUDE")) {
	  vector<int> indices;
	  indices.reserve(dataset.size());
	  int label = atoi(getenv("EXCLUDE"));
	  for(size_t i = 0; i < dataset.size(); ++i)
	    if(dataset.label(i)(label) <= 0)
	      indices.push_back(i);
	  tb.merge(dataset, indices);
	}
	else
	  tb.merge(dataset);
	
	cout << "Training buffer: " << endl;
	cout << tb.status("  ") << endl;
      }
    } while(getenv("INFINITE"));
  }
}

TEST(SplitDataset, SplitDataset)
{
  SyntheticDataGenerator sdg(getDefaultDimensionality(5), 10, 1, 0.1, defaultClassMap(), getStubDescriptorMap(5));
  int max_track_length = 1000;
  
  {
    TrackDataset::Ptr td = sdg.sampleTrackDataset(50, max_track_length);
    for(size_t i = 0; i < td->size(); ++i) {
      Dataset& track = (*td)[i];
      int num = rand() % (max_track_length - 1);
      track.instances_.erase(track.instances_.end() - num, track.instances_.end());
      assert(!track.instances_.empty());
    }
    cout << "Before cropping: " << endl;
    cout << td->status("  ");
    {
      ScopedTimer st("cropTracks");
      cropTracks(100, td.get());
    }
    cout << "After cropping: " << endl;
    cout << td->status("  ");

    for(size_t i = 0; i < td->size(); ++i)
      EXPECT_TRUE(td->tracks_[i]->size() <= 100);
  }
  
  {
    TrackDataset::Ptr td = sdg.sampleTrackDataset(50, max_track_length);
    for(size_t i = 0; i < td->size(); ++i) {
      Dataset& track = (*td)[i];
      int num = rand() % (max_track_length - 1);
      track.instances_.erase(track.instances_.end() - num, track.instances_.end());
      assert(!track.instances_.empty());
    }
    size_t total_instances = td->totalInstances();
    cout << "Before splitting: " << endl;
    cout << td->status("  ");
    {
      ScopedTimer st("splitTracks");
      splitTracks(100, td.get());
    }
    cout << "After splitting: " << endl;
    cout << td->status("  ");

    EXPECT_EQ(td->totalInstances(), total_instances);
    for(size_t i = 0; i < td->size(); ++i)
      EXPECT_TRUE(td->tracks_[i]->size() <= 100);
  }

  {
    
    TrackDataset::Ptr td = sdg.sampleTrackDataset(50, max_track_length);
    for(size_t i = 0; i < td->size(); ++i) {
      Dataset& track = (*td)[i];
      int num = rand() % (max_track_length - 1);
      track.instances_.erase(track.instances_.end() - num, track.instances_.end());
      assert(!track.instances_.empty());
    }
    size_t total_instances = td->totalInstances();
    cout << "Before splitting: " << endl;
    cout << td->status("  ");
    {
      ScopedTimer st("splitTracksFixedLength");
      splitTracksFixedLength(100, td.get());
    }
    cout << "After splitting: " << endl;
    cout << td->status("  ");

    EXPECT_TRUE(td->totalInstances() <= total_instances);
    for(size_t i = 0; i < td->size(); ++i)
      EXPECT_TRUE(td->tracks_[i]->size() == 100);
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
