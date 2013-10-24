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

float computeSURF(cv::Mat3b img)
{
  cv::SURF surf(1000);
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

void loadData(std::string data_dir, int maxnum, Eigen::MatrixXd* X, Eigen::VectorXd* y)
{
  // -- Get the number of labeled instances and their paths.
  vector<string> paths = glob(data_dir + "/*.png");
  if(maxnum > 0 && paths.size() > (size_t)maxnum)
    paths.resize(maxnum);
  *y = VectorXd(paths.size());
  *X = MatrixXd(1, paths.size());
  
  for(size_t i = 0; i < paths.size(); ++i) {
    // -- Load image and set the label.
    cout << "Loading " << paths[i] << endl;
    cv::Mat3b img = cv::imread(paths[i]);
    string numstr = paths[i].substr(paths[i].find_last_of("-") + 1);
    numstr = numstr.substr(0, numstr.size() - 4);
    y->coeffRef(i) = atof(numstr.c_str());
    
    // -- Compute the features.
    X->coeffRef(0, i) = computeSURF(img);
    //X->coeffRef(1, i) = deltaImage(img);
  }
}

int main(int argc, char** argv)
{
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  string data_dir;
  int maxnum;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("data", bpo::value(&data_dir)->required(), "")
    ("maxnum", bpo::value(&maxnum)->default_value(0), "")
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
  loadData(data_dir, maxnum, &X, &y);

  for(int i = 0; i < y.rows(); ++i)
    cout << X.col(i).transpose() << " -- " << y(i) << endl;

  mpliBegin();
  mpli("from pylab import *");
  mpliPrintSize();
  mpliExport(X);
  mpliExport(y);
  mpli("print X[0, :]");
  mpli("print y");
  mpli("scatter(X[0, :], y)");
  mpli("xlabel('SURF feature count')");
  mpli("ylabel('Ground truth nut count')");
  mpli("draw()");
  mpli("savefig('scatterplot.png')");
  mpli("clf()");
  
  return 0;
}
