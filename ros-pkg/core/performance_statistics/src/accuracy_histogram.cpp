#include <performance_statistics/performance_statistics.h>

using namespace std;
using namespace Eigen;

AccuracyHistogram::AccuracyHistogram(string xlabel, string ylabel, double bin_size,
				     string title, double xmin, double xmax) :
  max_y_(0),
  xlabel_(xlabel),
  ylabel_(ylabel),
  bin_size_(bin_size),
  title_(title),
  xmin_(xmin),
  xmax_(xmax)
{
  if(xmin != -FLT_MAX && xmax != FLT_MAX) { 
    double nbins = (xmax - xmin) / bin_size;
    assert(fabs(nbins - round(nbins)) < 1e-6); // Make sure that there are roughly an integral numbers of bins.  Otherwise the last bin is artificially small.
  }
}

void AccuracyHistogram::insert(int label, int prediction, double value) {
  if(value > xmax_ || value < xmin_)
    return;
  
  if(label == prediction)
    correct_.push_back(value);
  else
    incorrect_.push_back(value);
}

void AccuracyHistogram::saveHistogram(const std::string& filename) const {
  // -- Get the min and max values.
  double min = FLT_MAX;
  double max = -FLT_MAX;

  for(size_t i = 0; i < correct_.size(); ++i) {
    if(correct_[i] > max)
      max = correct_[i];
    if(correct_[i] < min)
      min = correct_[i];
  }

  for(size_t i = 0; i < incorrect_.size(); ++i) {
    if(incorrect_[i] > max)
      max = incorrect_[i];
    if(incorrect_[i] < min)
      min = incorrect_[i];
  }

  // -- Build vector of indices, starting with the minimum value of the first bin
  //    and ending with the minimum value of the last bin.
  int num_bins = ceil((max - min) / bin_size_);
  if(min == FLT_MAX || num_bins == 0) {
    ROS_WARN_STREAM("Tried to produce AccuracyHistogram \"" << filename << "\", but there was no data.  Not saving...");
    return;
  }

  VectorXd indices(num_bins);
  for(int i = 0; i < indices.rows(); ++i) {
    indices(i) = min + i * bin_size_;
  }

  // -- Put all the data into bins.
  VectorXd correct_bins = VectorXd::Zero(indices.rows());
  for(size_t i = 0; i < correct_.size(); ++i) {
    int idx = floor((correct_[i] - min) / bin_size_);
    if(idx == num_bins) // Extreme edge case
      --idx;
    ++correct_bins(idx);
  }

  VectorXd incorrect_bins = VectorXd::Zero(indices.rows());
  for(size_t i = 0; i < incorrect_.size(); ++i) {
    int idx = floor((incorrect_[i] - min) / bin_size_);
    if(idx == num_bins)
      --idx;
    ++incorrect_bins(idx);
  }

  // -- Get percent correct.
  assert(indices.rows() == correct_bins.rows());
  assert(correct_bins.rows() == incorrect_bins.rows());
  assert(indices.rows() > 0);
  VectorXd percents = VectorXd::Zero(indices.rows());
  for(int i = 0; i < correct_bins.rows(); ++i) {
    if(correct_bins(i) + incorrect_bins(i) != 0)
      percents(i) = correct_bins(i) / (correct_bins(i) + incorrect_bins(i));
  }
  percents *= 100.0;
  
  // -- Draw bars.
  mpliBegin();
  mpli("import matplotlib.pyplot as plt");
  mpli("import numpy as np");
  mpli("import matplotlib");
  mpliPrintSize();
  
  mpliExport(indices);
  mpliExport(bin_size_);
  mpli("fig = plt.figure()");
  mpli("ax1 = fig.add_subplot(111)");
  mpli("ax2 = ax1.twinx()");

  mpliExport(correct_bins);
  mpliExport(incorrect_bins);
  mpli("p1 = ax1.bar(indices, correct_bins, bin_size_, color=(0, 0.3, 0))");
  mpli("p2 = ax1.bar(indices, incorrect_bins, bin_size_, color=(0.85, 0.85, 0.85), bottom=correct_bins)");

  // -- Plot only parts of the percent line that have data.
  vector< vector<double> > percents_parts(1);
  vector< vector<double> > indices_parts(1);
  for(int i = 0; i < percents.rows(); ++i) {
    if(percents(i) > 0.005) {
      percents_parts.back().push_back(percents(i));
      indices_parts.back().push_back(indices(i) + bin_size_ / 2.0);
    }
    else {
      percents_parts.push_back(vector<double>());
      indices_parts.push_back(vector<double>());
    }
  }
  for(size_t i = 0; i < percents_parts.size(); ++i) {
    if(percents_parts[i].empty())
      continue;
    mpliNamedExport("percents", percents_parts[i]);
    mpliNamedExport("indices", indices_parts[i]);
    if(i == 0)
      mpli("p3 = ax2.plot(indices, percents, 'r*--')");
    else
      mpli("ax2.plot(indices, percents, 'r*--')");
  }

  // -- Window dressing and formatting.
  mpli("plt.legend((p1[0], p2[0], p3[0]), ('Correct', 'Incorrect', 'Percent'), loc='center left')"); // Why does 'best' always suck so badly?
  mpliExport(xlabel_);
  mpliExport(ylabel_);
  mpli("ax1.set_xlabel(xlabel_, fontsize=20)");
  mpli("ax1.set_ylabel(ylabel_, fontsize=20)");
  mpli("ax2.set_ylabel('Percent', fontsize=20)");

  if(max_y_ != 0) {
    mpliExport(max_y_);
    mpli("ax1.set_ylim(0, max_y_)");
  }
  mpli("ax2.set_ylim(0, 100)");
  mpli("ax2.set_yticks(np.arange(20, 120, 20))");
  mpliExport(xmin_);
  mpliExport(xmax_);
  mpli("ax1.set_xlim(xmin_, xmax_)");
  mpli("ax2.set_xlim(xmin_, xmax_)");

  // -- Save.
  mpliExport(filename);
  //mpli("print 'Saving to ' + filename");
  mpliExport(title_);
  mpli("plt.title(title_)");
  mpli("plt.savefig(filename)");
  mpli("plt.clf()");
}

