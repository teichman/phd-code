#include <performance_statistics/performance_statistics.h>
#include <gtest/gtest.h>

using namespace std;
using namespace Eigen;


double sampleFromGaussian(double stdev) {
  double val = 0;
  for(int i = 0; i < 12; ++i) { 
    double uniform = (double)(rand() % 1000) / 1000.0;
    val += (uniform * 2.0 - 1.0) * stdev;
  }
  return val / 2.0;
}

NameMapping getNameMapping() {
  NameMapping cmap;
  cmap.addName("car");
  cmap.addName("pedestrian");
  cmap.addName("bicyclist");
  return cmap;
}

VectorXf getRandomResponse(int length, int label) {
  VectorXf response(length);
  for(int c = 0; c < length; ++c) { 
    double y = -1;
    if(label == c)
      y = 1;
    
    if(rand() % 10 > 1)
      response(c) = y * (double)(rand() % 100) / 20.;
    else
      response(c) = -y * (double)(rand() % 100) / 20.;
  }
  return response;
}


TEST(AccuracyHistogram, sampleAccuracyHistogram) {
  AccuracyHistogram ah("P(Y=1 | x)", "Number of test examples", 0.05, "", 0, 1);
  ah.max_y_ = 80000;
  int num_samples = 1e6;
  for(int i = 0; i < num_samples; ++i) {
    int label;
    double val;
    if(rand() % 2) {
      label = 1;
      val = sqrt((double)rand() / RAND_MAX);
    }
    else {
      label = -1;
      val = (double)rand() / RAND_MAX;
      val = val * val;
    }
    int pred = -1;
    if(val > 0.5)
      pred = 1;
    ah.insert(label, pred, val);
  }

  ah.saveHistogram("sample_accuracy_histogram.png");
  ah.saveHistogram("sample_accuracy_histogram.pdf");
}


TEST(PerfStats, sampleConfusionMatrix) {
  PerfStats stats;
  stats.applyNameMapping("cmap", getNameMapping());

  for(int i = 0; i < 1000; ++i) {
    int label = (rand() % (stats.nameMapping("cmap").size() + 1)) - 1;
    VectorXf response = getRandomResponse(stats.nameMapping("cmap").size(), label);
    string label_str;
    if(label == -1)
      label_str = "background";
    else
      label_str = stats.nameMapping("cmap").toName(label);
    stats.incrementStats(label_str, response);
  }

  cout << stats.statString() << endl;
  stats.saveConfusionMatrix("sample_confusion_matrix.pdf");
  stats.saveConfusionMatrix("sample_confusion_matrix.png");
  stats.savePrecisionRecallCurve("sample_pr_curve.png");
}

TEST(PerfStats, PR_stress_test)
{
  PerfStats stats;
  stats.applyNameMapping("cmap", getNameMapping());

  for(int i = 0; i < 100000; ++i) {
    int label = (rand() % (stats.nameMapping("cmap").size() + 1)) - 1;
    VectorXf response = getRandomResponse(stats.nameMapping("cmap").size(), label);
    for(int j = 0; j < response.rows(); ++j)
      if(fabs(response(j)) < 1.5)
	response(j) = 0;
    
    string label_str;
    if(label == -1)
      label_str = "background";
    else
      label_str = stats.nameMapping("cmap").toName(label);
    stats.incrementStats(label_str, response);
  }

  stats.savePrecisionRecallCurve("stress_pr_curve.png");
  stats.saveAccuracyVsConfidence("acc_vs_conf", "Accuracy vs confidence", 100);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
