#include <matplotlib_interface/matplotlib_interface.h>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <Eigen/Eigen>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <bag_of_tricks/glob.h>


using namespace std;
using namespace Eigen;
namespace bpo = boost::program_options;
namespace bfs = boost::filesystem;

float computeSURF(bpo::variables_map opts, cv::Mat3b img)
{
  cv::SURF surf(opts["surf-hessian-threshold"].as<double>(),
                opts["surf-num-octaves"].as<int>(),
                opts["surf-num-octave-layers"].as<int>());
  vector<cv::KeyPoint> keypoints;
  surf(img, cv::Mat(), keypoints);
  return keypoints.size();
}

// float deltaImage(cv::Mat3b img, cv::Mat3b prev_img)
// {
//   cv::Mat1b delta = cv::Mat1b(img.size(), 0);
//   for(int y = 0; y < curr.rows; ++y)
//     for(int x = 0; x < curr.cols; ++x)
//       delta(y, x) = fabs(curr(y, x) - prev(y, x));
// }

double stdev(const Eigen::VectorXd& vec)
{
  double mean = vec.sum() / vec.rows();
  double total = 0;
  for(int i = 0; i < vec.rows(); ++i)
    total += pow(vec(i) - mean, 2);

  return sqrt(total / vec.rows());
}

float differenceFeature(cv::Mat1b img)
{
  float diff = 0;
  for(int y = 0; y < img.rows; ++y)
    for(int x = 0; x < img.cols; ++x)
      diff += img(y, x);
  diff /= (img.rows * img.cols);
  return diff;
}

void loadData(std::string data_dir, std::string difference_image_dir,
              bpo::variables_map opts,
              int maxnum, Eigen::MatrixXd* X, Eigen::VectorXd* y, std::vector<string>* paths)
{
  int num_features = 6;
  
  // -- Get the number of labeled instances and their paths.
  *paths = glob(data_dir + "/*.png");
  if(maxnum > 0 && paths->size() > (size_t)maxnum)
    paths->resize(maxnum);
  *y = VectorXd(paths->size());
  *X = MatrixXd(num_features, paths->size());
  
  for(size_t i = 0; i < paths->size(); ++i) {
    // -- Load image and set the label.
    cout << "Loading " << (*paths)[i] << endl;
    cv::Mat3b img = cv::imread((*paths)[i]);
    string numstr = (*paths)[i].substr((*paths)[i].find_last_of("-") + 1);
    numstr = numstr.substr(0, numstr.size() - 4);
    y->coeffRef(i) = atof(numstr.c_str());

    // -- Load the delta image.
    string filename = (*paths)[i].substr((*paths)[i].find_last_of('/') + 1);
    string framestr = filename.substr(0, filename.find("raw"));
    string difference_image_path = difference_image_dir + "/difference" + framestr + ".png";
    cv::Mat1b diff = cv::imread(difference_image_path, CV_LOAD_IMAGE_GRAYSCALE);
    for(int y = 0; y < diff.rows; ++y) {
      for(int x = 0; x < diff.cols; ++x) {
        if(diff(y, x) > opts["difference-image-threshold"].as<float>())
          diff(y, x) = 255;
        else
          diff(y, x) = 0;
      }
    }

    X->coeffRef(0, i) = 1;
    X->coeffRef(1, i) = computeSURF(opts, img);
    X->coeffRef(2, i) = X->coeffRef(1, i) * X->coeffRef(1, i);
    X->coeffRef(3, i) = differenceFeature(diff);
    X->coeffRef(4, i) = X->coeffRef(3, i) * X->coeffRef(3, i);
    X->coeffRef(5, i) = X->coeffRef(1, i) * X->coeffRef(3, i);
  }
}

int main(int argc, char** argv)
{
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  string data_dir;
  string difference_image_dir;
  int maxnum;
  int plot_index;
  float difference_image_threshold;
  string feature_string;
  double surf_hessian_threshold;
  int surf_num_octaves;
  int surf_num_octave_layers;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("data", bpo::value(&data_dir)->required(), "")
    ("difference-image-dir", bpo::value(&difference_image_dir)->required(), "")
    ("maxnum", bpo::value(&maxnum)->default_value(0), "")
    ("plot", bpo::value(&plot_index), "Which index to plot, if any")
    ("difference-image-threshold", bpo::value(&difference_image_threshold)->default_value(5), "")
    ("feature-string", bpo::value(&feature_string)->default_value("Feature"), "")
    ("surf-hessian-threshold", bpo::value(&surf_hessian_threshold)->default_value(100), "")
    ("surf-num-octaves", bpo::value(&surf_num_octaves)->default_value(4), "")
    ("surf-num-octave-layers", bpo::value(&surf_num_octave_layers)->default_value(2), "")
    ;

  p.add("data", 1);
  
  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << argv[0] << " [OPTS] DATA_DIR" << endl; 
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  cout << "Loading data at \"" << data_dir << "\"." << endl;
  MatrixXd X;
  VectorXd y;
  vector<string> paths;
  loadData(data_dir, difference_image_dir, opts, maxnum, &X, &y, &paths);

  for(int i = 0; i < y.rows(); ++i)
    cout << X.col(i).transpose() << " -- " << y(i) << endl;

  // -- Plot data.
  if(opts.count("plot")) {
    mpliBegin();
    mpli("from pylab import *");
    mpliPrintSize();
    mpliExport(X);
    mpliExport(y);
    mpliExport(plot_index);
    mpliExport(feature_string);
    mpli("print X[plot_index, :]");
    mpli("print y");
    mpli("scatter(X[plot_index, :], y)");
    mpli("xlabel(feature_string)");
    mpli("ylabel('Ground truth nut count')");
    mpli("xlim(xmin=0)");
    mpli("draw()");
    mpli("savefig('scatterplot.png')");
    mpli("clf()");
  }

  // -- Leave one out cross-validation.
  VectorXd absolute_errors(y.rows());
  VectorXd signed_errors(y.rows());
  VectorXd percent_errors(y.rows());
  VectorXd predictions(y.rows());
  for(int i = 0; i < y.rows(); ++i) {
    // Construct the training set.
    MatrixXd X2(X.rows(), X.cols() - 1);
    VectorXd y2(y.rows() - 1);
    int idx = 0;
    for(int j = 0; j < y.rows(); ++j) {
      if(j != i) {
        X2.col(idx) = X.col(j);
        y2(idx) = y(j);
        ++idx;
      }
    }

    // Fit the model.
    VectorXd weights = (X2 * X2.transpose()).inverse() * X2 * y2;
    cout << "weights: " << weights.transpose();
    cout << ", x: " << X.col(i).transpose();

    // Evaluate on the held-out test example.
    double yhat = weights.dot(X.col(i));
    cout << ", prediction: " << yhat << ", L1 error: " << fabs(yhat - y(i));
    cout << endl;

    predictions(i) = yhat;
    absolute_errors(i) = fabs(yhat - y(i));
    signed_errors(i) = yhat - y(i);
    percent_errors(i) = fabs(yhat - y(i)) / max<double>(y(i), 1);
  }

  cout << endl;
  cout << "Mean count error: " << absolute_errors.sum() / absolute_errors.rows() << endl;
  cout << "Standard deviation of count error: " << stdev(absolute_errors) << endl;
  cout << "Mean percent error: " << percent_errors.sum() / percent_errors.rows() << endl;
  cout << "Standard deviation of percent error: " << stdev(percent_errors) << endl;

  vector< pair<double, size_t> > index;
  index.resize(absolute_errors.rows());
  for(int i = 0; i < absolute_errors.rows(); ++i) {
    index[i].first = absolute_errors(i);
    index[i].second = i;
  }
  sort(index.begin(), index.end(), greater< pair<double, size_t> >());  // descending

  int k = 5;
  cout << "Worst " << k << " test examples: " << endl;
  for(int i = 0; i < k; ++i) {
    size_t idx = index[i].second;
    cout << "Prediction: " << predictions(idx) << ", Ground truth: " << y(idx) << ", Error: " << signed_errors(idx) << ", path: " << paths[idx] << endl;
  }
  
  return 0;
}
