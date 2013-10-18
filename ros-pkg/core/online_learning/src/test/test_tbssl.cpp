#include <online_learning/tbssl.h>
#include <gtest/gtest.h>
#include <gperftools/profiler.h>
#include <online_learning/synthetic_data_generator.h>

using namespace std;
using namespace Eigen;
namespace bfs = boost::filesystem;

TEST(TBSSL, minCoeff)
{
  ArrayXf numerator(2);
  numerator << 13, 1;
  ArrayXf denominator(2);
  denominator << 0, 13;

  EXPECT_NEAR((numerator / denominator).minCoeff(), 1/13., 1e-6);
}

TEST(TBSSL, Resumption)
{
  srand(0);
  
  string dir = "tbssl_automated_resumption_test";
  string unlabeled_dir = dir + "/unlabeled";
  string output_path = dir + "/output";
  int num_unl = 20;
  bfs::remove_all("tbssl_automated_resumption_test");
  bfs::create_directory(dir);
  bfs::create_directory(unlabeled_dir);
  bfs::create_directory(output_path);
  
  // -- Fill out unlabeled dir with datasets.
  SyntheticDataGenerator sdg = getDefaultGenerator(3);
  SyntheticDataGenerator bgg = getDefaultGenerator(3);  // Different means.
  for(int i = 0; i < num_unl; ++i) {
    TrackDataset::Ptr unlabeled = sdg.sampleTrackDataset(500, 7);
    TrackDataset::Ptr bg = bgg.sampleTrackDataset(500, 7);  // Synthetic background data.  Just other stuff in the world that we don't care about.
    for(size_t j = 0; j < bg->size(); ++j) {
      Label label = bg->label(j);
      label.setZero();
      (*bg)[j].setLabel(label);
    }
    ostringstream oss;
    oss << unlabeled_dir << "/unl" << setw(2) << setfill('0') << i << ".td";
    unlabeled->save(oss.str());
  }

  // -- Generate other data.
  TrackDataset::Ptr init = sdg.sampleTrackDataset(1000, 20);
  TrackDataset::Ptr test = sdg.sampleTrackDataset(1000, 1);
  TrackDataset::Ptr seed;
  if(getenv("GIANT_SEED")) {
    ROS_WARN("Using giant seed dataset.");
    seed = sdg.sampleTrackDataset(6, 5000);
  }
  else 
    seed = sdg.sampleTrackDataset(6, 3);
  TrackDataset::Ptr bg = bgg.sampleTrackDataset(2000, 1);  
  for(size_t i = 0; i < bg->size(); ++i) {
    Label label = bg->label(i);
    label.setConstant(-1);
    (*bg)[i].setLabel(label);
  }
      
  cout << "Init dataset: " << endl;
  cout << init->status("  ") << endl;
  cout << "Test dataset: " << endl;
  cout << test->status("  ") << endl;
  cout << "Seed dataset: " << endl;
  cout << seed->status("  ") << endl;
  cout << "Background dataset: " << endl;
  cout << bg->status("  ") << endl;

  // -- Set up the classifier.
  GridClassifier::Ptr gc(new GridClassifier);
  vector<size_t> num_cells;
  num_cells.push_back(100);
  num_cells.push_back(20);
  gc->initialize(*init, num_cells);

  // -- Initialize and run the first iteration.
  GridClassifier::BoostingTrainer::Ptr trainer(new GridClassifier::BoostingTrainer(gc));
  trainer->verbose_ = true;
  trainer->gamma_ = 0;
  OnlineLearner learner(0, 5000, 5, gc, trainer, 1, 1, 1, output_path, unlabeled_dir);
  learner.setTestData(test);
  //seed->setImportance(1e30);
  learner.pushHandLabeledDataset(seed->clone());
  //learner.pushAutoLabeledDataset(bg);
  learner.run();

  // -- Evaluate.
  GridClassifier::Ptr gc_tmp(new GridClassifier);
  learner.copyClassifier(gc_tmp.get());
  Evaluator ev(gc_tmp);
  ev.evaluate(*test);
  cout << "Test set frame accuracy: " << ev.frame_stats_.getTotalAccuracy() << endl;
  cout << "Test set track accuracy: " << ev.track_stats_.getTotalAccuracy() << endl;
  Evaluator ev_seed(gc_tmp);
  ev_seed.evaluate(*seed);
  cout << "Seed frame accuracy: " << ev_seed.frame_stats_.getTotalAccuracy() << endl;
  cout << "Seed track accuracy: " << ev_seed.track_stats_.getTotalAccuracy() << endl;

  vector<float> accuracy;
  accuracy.push_back(ev.frame_stats_.getTotalAccuracy());
  
  // -- Now run for a while, serializing every time.
  for(int i = 0; i < num_unl + 5; ++i) {
    cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
    
    OnlineLearner learner((IfstreamWrapper(output_path + "/learner.ol")));
    if(i == 2) {
      learner.pushHandLabeledDataset(sdg.sampleTrackDataset(6, 3));
    }
    if(i == 4) {
      // bg->save(output_path + "/input_auto_annotations/bg.td");
      // bg->save(output_path + "/input_hand_annotations/bg.td");
      TrackDataset::Ptr seed3 = sdg.sampleTrackDataset(6, 3);
      seed3->save(output_path + "/input_hand_annotations/seed.td");
    }
    
    learner.setTestData(test);
    learner.setMaxIters(2+i);
    learner.run();

    learner.copyClassifier(gc_tmp.get());
    Evaluator ev(gc_tmp);
    ev.evaluate(*test);
    cout << "Test set frame accuracy: " << ev.frame_stats_.getTotalAccuracy() << endl;
    cout << "Test set track accuracy: " << ev.track_stats_.getTotalAccuracy() << endl;
    Evaluator ev_seed(gc_tmp);
    ev_seed.evaluate(*seed);
    cout << "Seed frame accuracy: " << ev_seed.frame_stats_.getTotalAccuracy() << endl;
    cout << "Seed track accuracy: " << ev_seed.track_stats_.getTotalAccuracy() << endl;
    
    accuracy.push_back(ev.frame_stats_.getTotalAccuracy());
  }
    
  cout << "Accuracies: " << endl;
  for(size_t i = 0; i < accuracy.size(); ++i) {
    cout << accuracy[i] << endl;
    if(i > 0)
      EXPECT_TRUE((accuracy[i] + 0.01) >= accuracy[i-1]);
  }
  EXPECT_TRUE(accuracy.back() > 0.9);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
