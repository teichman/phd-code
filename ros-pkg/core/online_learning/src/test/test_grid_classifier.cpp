#include <online_learning/evaluator.h>

#include <gtest/gtest.h>
#include <gperftools/profiler.h>
#include <online_learning/grid_classifier.h>
#include <online_learning/synthetic_data_generator.h>
#include <online_learning/training_buffer.h>

using namespace std;
using namespace Eigen;

#define NUM_PARALLEL_EVALS (getenv("NUM_PARALLEL_EVALS") ? atoi(getenv("NUM_PARALLEL_EVALS")) : 1)

TEST(GridClassifierBoostingTrainer, Labels)
{
  Label label = VectorXf::Zero(4);
  label(0) = -42;
  label(2) = +42;
  label(3) = -numeric_limits<float>::min();
  ArrayXd array = label.sign().array().cast<double>();  // This is assumed to produce a vector in -1, 0, +1.
  cout << array.transpose() << endl;
  EXPECT_FLOAT_EQ(array(0), -1);
  EXPECT_FLOAT_EQ(array(1), 0);
  EXPECT_FLOAT_EQ(array(2), 1);
  EXPECT_FLOAT_EQ(array(3), -1);
}

TEST(Grid, NaN)
{
  float val = std::numeric_limits<float>::quiet_NaN();
  int num_cols = 20;
  float min = 0.13;
  float inv_width_ = 0.42;
  int idx = std::min(num_cols - 1, std::max(0, (int)((val - min) * inv_width_)));
  cout << "Cell index for NaN: " << idx << endl;
  EXPECT_EQ(idx, 0);

  val = -std::numeric_limits<float>::quiet_NaN();
  idx = std::min(num_cols - 1, std::max(0, (int)((val - min) * inv_width_)));
  cout << "Cell index for -NaN: " << idx << endl;
  EXPECT_EQ(idx, 0);
}

TEST(sign, sign)
{
  EXPECT_TRUE(sign(numeric_limits<float>::min()) == 1);
  EXPECT_TRUE(sign(numeric_limits<float>::max()) == 1);
  EXPECT_TRUE(sign(-numeric_limits<float>::min()) == -1);
  EXPECT_TRUE(sign(0) == 0);
  EXPECT_TRUE(sign(0.0) == 0);

  Label label = VectorXf::Zero(3);
  label.setConstant(-10);
  EXPECT_TRUE((label.sign().array() == -1).all());
  EXPECT_TRUE((label.array().abs() == 10).all());
  label(1) = 0;
  EXPECT_TRUE(label.sign()(1) == 0);
  EXPECT_TRUE(label.array().abs()(1) == 0);
}

TEST(GridClassifier, GridClassifier)
{
  SyntheticDataGenerator sdg = getDefaultGenerator(10);
  TrackDataset::Ptr train = sdg.sampleTrackDataset(20, 5);
  
  GridClassifier::Ptr gc(new GridClassifier);
  vector<size_t> num_cells;
  num_cells.push_back(50);
  num_cells.push_back(500);
  gc->initialize(*train, num_cells);
  GridClassifier::StochasticLogisticTrainer trainer(gc);
  TrainingBuffer buffer;
  buffer.applyNameMappings(*gc);
  buffer.init(100);
  buffer.merge(*train);

  vector<TrackDataset::ConstPtr> ds;
  ds.push_back(buffer.getTrackDataset());
  ds.push_back(train);
  vector<Indices> indices;
  for(size_t i = 0; i < ds.size(); ++i)
    indices.push_back(Indices::All(ds[i]->size()));
  HighResTimer hrt("Serial training");
  hrt.start();
  ProfilerStart("test_training_serial.prof");
  trainer.trainSerial(ds, indices);  // empty -> use all
  ProfilerStop();
  hrt.stop();
  cout << hrt.report() << endl;
  cout << "Prior: " << gc->prior().transpose() << endl;
  
  // -- Make sure we can get reasonable performance on an easy dataset.
  TrackDataset::Ptr test = sdg.sampleTrackDataset(10000, 2);
  Evaluator ev(gc);
  ev.evaluate(*test);
  cout << "Test set frame accuracy: " << ev.frame_stats_.getTotalAccuracy() << endl;
  cout << "Test set track accuracy: " << ev.track_stats_.getTotalAccuracy() << endl;
  EXPECT_TRUE(ev.frame_stats_.getTotalAccuracy() > 0.9);
  EXPECT_TRUE(ev.track_stats_.getTotalAccuracy() > 0.9);

  // -- Test evaluate vs evaluateParallel
  for(int i = 0; i < NUM_PARALLEL_EVALS; ++i) { 
    Evaluator ev2(gc);
    ev2.evaluateParallel(*test);
    cout << "Serial: " << ev.evaluation_timer_.getSeconds() << " seconds.  Parallel: " << ev2.evaluation_timer_.getSeconds() << " seconds." << endl;
    EXPECT_TRUE(ev.frame_stats_.getTotalAccuracy() < 1);  // If they're both saturated, it might not catch differences.
    EXPECT_TRUE(ev.frame_stats_.getTotalAccuracy() == ev2.frame_stats_.getTotalAccuracy());
    EXPECT_TRUE(ev.track_stats_.getTotalAccuracy() == ev2.track_stats_.getTotalAccuracy());
  }
  
  // -- Test copying.
  GridClassifier gc2 = *gc;
  cout << "Original: " << endl;
//  cout << gc->status("  ");
  cout << "Copy: " << endl;
//  cout << gc2.status("  ");
  EXPECT_TRUE(gc2 == *gc);

  // -- Test saving.
  gc->save("test_classifier.gc");
  GridClassifier::Ptr gc3(new GridClassifier);
  gc3->load("test_classifier.gc");
  EXPECT_TRUE(*gc == *gc3);

  trainer.save("test_trainer.gct");
  GridClassifier::StochasticLogisticTrainer trainer3(gc3);
  trainer3.load("test_trainer.gct");
  cout << "Original trainer: " << endl;
  cout << trainer.status() << endl;
  cout << "Loaded trainer: " << endl;
  cout << trainer3.status() << endl;
  EXPECT_TRUE(trainer == trainer3);

  // -- Test parallel training.
  {
    cout << "**** MiniBatch test" << endl;
    GridClassifier::Ptr gc(new GridClassifier);
    gc->initialize(*train, num_cells);
    GridClassifier::StochasticLogisticTrainer trainer(gc);

    HighResTimer hrt("MiniBatch training");
    hrt.start();
    ProfilerStart("test_training_minibatch.prof");
    trainer.train(ds, indices);  // empty -> use all
    ProfilerStop();
    hrt.stop();
    cout << hrt.report() << endl;
    cout << "Prior: " << gc->prior().transpose() << endl;

    Evaluator ev(gc);
    ev.evaluateParallel(*test);
    cout << "Test set frame accuracy: " << ev.frame_stats_.getTotalAccuracy() << endl;
    cout << "Test set track accuracy: " << ev.track_stats_.getTotalAccuracy() << endl;
    EXPECT_TRUE(ev.frame_stats_.getTotalAccuracy() > 0.9);
    EXPECT_TRUE(ev.track_stats_.getTotalAccuracy() > 0.9);
  }
}

TEST(GridClassifier, GentleBoostingTrainer)
{
  SyntheticDataGenerator sdg = getDefaultGenerator(5);
  TrackDataset::Ptr train = sdg.sampleTrackDataset(20, 10);
  TrackDataset::Ptr unl = sdg.sampleTrackDataset(100, 100);
  for(size_t i = 0; i < unl->size(); ++i)
    (*unl)[i].setLabel(VectorXf::Zero(unl->label(i).rows()));
  cout << unl->status() << endl;
  *train += *unl;
  
  GridClassifier::Ptr gc(new GridClassifier);
  vector<size_t> num_cells;
  num_cells.push_back(50);
  num_cells.push_back(500);
  gc->initialize(*train, num_cells);
  GridClassifier::BoostingTrainer trainer(gc);
  trainer.gamma_ = 0.5;
  trainer.verbose_ = true;
  TrainingBuffer buffer;
  buffer.applyNameMappings(*gc);
  buffer.init(100);
  buffer.merge(*train);

  vector<TrackDataset::ConstPtr> ds;
  ds.push_back(buffer.getTrackDataset());
  ds.push_back(train);
  vector<Indices> indices;
  indices.push_back(Indices::All(ds[0]->size()));
  indices.push_back(Indices::All(ds[1]->size()));
  HighResTimer hrt("Boosting training");
  hrt.start();
  ProfilerStart("test_boosting_training.prof");
  trainer.train(ds, indices);
  ProfilerStop();
  hrt.stop();
  cout << hrt.report() << endl;
  
  // -- Make sure we can get reasonable performance on an easy dataset.
  TrackDataset::Ptr test = sdg.sampleTrackDataset(10000, 2);
  Evaluator ev(gc);
  ev.evaluate(*test);
  cout << "Test set frame accuracy: " << ev.frame_stats_.getTotalAccuracy() << endl;
  cout << "Test set track accuracy: " << ev.track_stats_.getTotalAccuracy() << endl;
  EXPECT_TRUE(ev.frame_stats_.getTotalAccuracy() > 0.9);
  EXPECT_TRUE(ev.track_stats_.getTotalAccuracy() > 0.9);

  // -- Test serialization.
  string trainer_filename = ".tmp_boosting_trainer";
  trainer.save(trainer_filename);
  GridClassifier::BoostingTrainer trainer2(gc);
  trainer2.load(trainer_filename);
  cout << trainer.status() << endl;
  cout << trainer2.status() << endl;
  EXPECT_TRUE(trainer == trainer2);    
}

// TEST(GridClassifier, BisectionBoostingTrainer)
// {
//   SyntheticDataGenerator sdg = getDefaultGenerator(20);
//   TrackDataset::Ptr train = sdg.sampleTrackDataset(20, 10);
  
//   GridClassifier::Ptr gc(new GridClassifier);
//   vector<size_t> num_cells;
//   num_cells.push_back(50);
//   num_cells.push_back(500);
//   gc->initialize(*train, num_cells);
//   GridClassifier::BisectionBoostingTrainer trainer(gc);
//   trainer.gamma_ = 0.5;
//   //trainer.verbose_ = true;
//   TrainingBuffer buffer;
//   buffer.applyNameMappings(*gc);
//   buffer.init(100);
//   buffer.merge(*train);

//   vector<TrackDataset::ConstPtr> ds;
//   ds.push_back(buffer.getTrackDataset());
//   ds.push_back(train);
//   vector<Indices> indices;
//   indices.push_back(Indices::All(ds[0]->size()));
//   indices.push_back(Indices::All(ds[1]->size()));
//   HighResTimer hrt("Boosting training");
//   hrt.start();
//   ProfilerStart("test_boosting_training.prof");
//   trainer.train(ds, indices);
//   ProfilerStop();
//   hrt.stop();
//   cout << hrt.report() << endl;
  
//   // -- Make sure we can get reasonable performance on an easy dataset.
//   TrackDataset::Ptr test = sdg.sampleTrackDataset(10000, 2);
//   Evaluator ev(gc);
//   ev.evaluate(*test);
//   cout << "Test set frame accuracy: " << ev.frame_stats_.getTotalAccuracy() << endl;
//   cout << "Test set track accuracy: " << ev.track_stats_.getTotalAccuracy() << endl;
//   EXPECT_TRUE(ev.frame_stats_.getTotalAccuracy() > 0.85);
//   EXPECT_TRUE(ev.track_stats_.getTotalAccuracy() > 0.9);

//   // -- Test serialization.
//   string trainer_filename = ".tmp_boosting_trainer";
//   trainer.save(trainer_filename);
//   GridClassifier::BisectionBoostingTrainer trainer2(gc);
//   trainer2.load(trainer_filename);
//   cout << trainer.status() << endl;
//   cout << trainer2.status() << endl;
//   EXPECT_TRUE(trainer == trainer2);    
// }

// TODO: Make this test use something other than the prior.
// TEST(GridClassifier, WeightedTraining)
// {
//   SyntheticDataGenerator sdg = getDefaultGenerator(20);
//   Dataset important = *sdg.sampleDataset(3000, 0);
//   Dataset boring = *sdg.sampleDataset(3000, 1);

//   // -- No weighting: priors should be about the same.
//   float dp_unweighted;
//   {
//     TrackDataset training;
//     training.applyNameMappings(important);
//     training.tracks_.push_back(important);
//     training.tracks_.push_back(boring);
//     cout << "Training dataset: " << endl;
//     cout << training.status("  ");
    
//     GridClassifier::Ptr gc(new GridClassifier);
//     vector<size_t> num_cells;
//     num_cells.push_back(50);
//     num_cells.push_back(500);
//     gc->initialize(training, num_cells);
//     GridClassifier::StochasticLogisticTrainer trainer(gc);
//     trainer.train(training);
    
//     VectorXf prior = gc->prior();
//     cout << "Unweighted training prior: " << prior.transpose() << endl;
//     dp_unweighted = fabs(prior(0) - prior(1));
//   }

//   // -- Strong weighting of class 0: the class 0 prior should be much stronger than the class 1 or 2 prior.
//   for(size_t i = 0; i < important.size(); ++i)
//     important[i].label_ *= 10;

//   float dp_weighted;
//   {
//     TrackDataset training;
//     training.applyNameMappings(important);
//     training.tracks_.push_back(important);
//     training.tracks_.push_back(boring);
    
//     GridClassifier::Ptr gc(new GridClassifier);
//     vector<size_t> num_cells;
//     num_cells.push_back(50);
//     num_cells.push_back(500);
//     gc->initialize(training, num_cells);
//     GridClassifier::StochasticLogisticTrainer trainer(gc);
//     trainer.train(training);
    
//     VectorXf prior = gc->prior();
    
//     cout << "Weighted training prior: " << prior.transpose() << endl;
//     dp_weighted = fabs(prior(0) - prior(1));
//   }
//   EXPECT_TRUE(dp_weighted > 5 * dp_unweighted);
// }

TEST(FastExp, FastExp)
{
  // for(double x = -100; x <= 100; x += 1) {
  //   double e = exp(x);
  //   double f = fastexp(x);
  //   cout << "x: " << x << ", exp: " << e << ", fastexp: " << f << ", relative error: " << fabs(f - e) / e << endl;
  // }


  {
    ScopedTimer st("Exp");
    double sum = 0;
    for(int i = 0; i < 1000; ++i) { 
      for(double x = -100; x <= 100; x += 1) {
        double e = exp(x);
        sum += e;
      }
    }
  }
  
  {
    ScopedTimer st("Fastexp");
    double sum = 0;
    for(int i = 0; i < 1000; ++i) { 
      for(double x = -100; x <= 100; x += 1) {
        double e = fastexp(x);
        sum += e;
      }
    }
  }

}

TEST(GentleBoostingTrainer, RealTest)
{
  if(getenv("DATASET")) {
    string path = getenv("DATASET");

    TrackDataset::Ptr train(new TrackDataset);
    train->load(path);

    GridClassifier::Ptr gc(new GridClassifier);
    vector<size_t> num_cells;
    num_cells.push_back(10);
    num_cells.push_back(100);
    gc->initialize(*train, num_cells);
    GridClassifier::BoostingTrainer trainer(gc);
    trainer.gamma_ = 0.1;
    trainer.verbose_ = true;

    vector<TrackDataset::ConstPtr> ds;
    ds.push_back(train);
    vector<Indices> indices;
    indices.push_back(Indices::All(ds[0]->size()));

    HighResTimer hrt("Boosting training");
    hrt.start();
    ProfilerStart("test_boosting_training.prof");
    trainer.train(ds, indices);
    ProfilerStop();
    hrt.stop();
    cout << hrt.reportSeconds() << endl;

    Evaluator ev(gc);
    ev.evaluate(*train);
    cout << "Training set set frame accuracy: " << ev.frame_stats_.getTotalAccuracy() << endl;
    cout << "Training set track accuracy: " << ev.track_stats_.getTotalAccuracy() << endl;
  }
}

TEST(GentleBoostingTrainer, LabelSwitch)
{
  if(!getenv("DATASET"))
    return;
  string path = getenv("DATASET");
  TrackDataset::Ptr train(new TrackDataset);
  train->load(path);

  GridClassifier::Ptr gc(new GridClassifier);
  vector<size_t> num_cells;
  num_cells.push_back(10);
  num_cells.push_back(100);
  gc->initialize(*train, num_cells);
  GridClassifier::BoostingTrainer trainer(gc);
  trainer.gamma_ = 0.5;
  trainer.verbose_ = true;
  
  vector<TrackDataset::ConstPtr> ds;
  ds.push_back(train);
  vector<Indices> indices;
  indices.push_back(Indices::All(ds[0]->size()));

  while(true) {     
    trainer.train(ds, indices);

    // -- Negate the training set.
    for(size_t i = 0; i < train->size(); ++i) {
      Label label = (*train)[i][0].label_;
      label *= -1;
      (*train)[i].setLabel(label);  // TODO: -label doesn't work.    label * -1 doesn't work.
    }
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
