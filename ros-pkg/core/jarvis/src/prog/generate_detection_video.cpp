#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <bag_of_tricks/glob.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <fstream>
#include <ros/assert.h>
#include <jarvis/jarvis.h>

using namespace std;
using namespace Eigen;
namespace bfs = boost::filesystem;

double logistic(double logodds, double sigma)
{
  return 1.0 / (1.0 + exp(-logodds / sigma));
}

struct Det
{
  double timestamp_;
  size_t track_id_;
  int ulx_;
  int uly_;
  int lrx_;
  int lry_;
  vector<float> frame_predictions_;
  vector<string> class_names_;
};

void load(string path, map<double, vector<Det> >* detections)
{
  ifstream file(path.c_str());
  
  while(true) {
    string line;
    getline(file, line);
    if(file.eof())
      break;

    Det det;
    istringstream iss(line);
    iss >> det.timestamp_;
    iss >> det.track_id_;
    iss >> det.ulx_;
    iss >> det.uly_;
    iss >> det.lrx_;
    iss >> det.lry_;

    while(!iss.eof()) {
      string class_name;
      iss >> class_name;
      float logodds;
      iss >> logodds;
      det.frame_predictions_.push_back(logodds);
      det.class_names_.push_back(class_name);
    }

    (*detections)[det.timestamp_].push_back(det);
  }
}

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  string images_dir;
  vector<string> detection_metadata_paths;
  string show;
  int pad;
  double sigma;
  double fps;
  string rotation;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("images-dir", bpo::value(&images_dir)->required(), "")
    ("detection-metadata-paths", bpo::value(&detection_metadata_paths)->required()->multitoken(), "")
    ("show", bpo::value(&show)->required(), "class to show detections for")
    ("pad", bpo::value(&pad)->default_value(7), "padding on the bounding box")
    ("sigma", bpo::value(&sigma)->default_value(5), "in the logistic function for coloring")
    ("fps", bpo::value(&fps)->default_value(30), "")
    ("rotation", bpo::value(&rotation)->default_value("rotate=1,rotate=1"), "mencoder rotation option")
    ("only-pos", "Show only positive detections rather than everything")
    ("frame", "Show frame predictions rather than track predictions")
    ;

  p.add("images-dir", 1);
  p.add("show", 1);
  p.add("detection-metadata-paths", -1);
  
  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << argv[0] << " [OPTS] IMAGES_DIR SHOW DETECTION_METADATA [ DETECTION_METADATA ... ]" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  // -- Load all detection metadata.
  map<double, vector<Det> > detections;
  for(size_t i = 0; i < detection_metadata_paths.size(); ++i) {
    cout << "Loading " << detection_metadata_paths[i] << endl;
    load(detection_metadata_paths[i], &detections);
  }
  
  string tmpdir = ".generate_detection_video-tmpdir";
  if(bfs::exists(tmpdir))
    bfs::remove_all(tmpdir);
  bfs::create_directory(tmpdir);

  std::map<size_t, DiscreteBayesFilter> filters;

  for(auto it = detections.begin(); it != detections.end(); ++it) {
    double timestamp = it->first;
    const vector<Det>& dets = it->second;

    // -- Update the filters.
    for(size_t j = 0; j < dets.size(); ++j) {
      const Det& det = dets[j];
      // Initialize if necessary.
      if(!filters.count(det.track_id_))
        filters[det.track_id_] = DiscreteBayesFilter(10, 1);
        
      // Update DiscreteBayesFilter.  Use fake centroid motion because we don't have
      // that info anymore.
      Label fpred(det.frame_predictions_);
      VectorXf centroid = VectorXf::Zero(3);
      centroid(0) = filters[det.track_id_].numObservations();
      filters[det.track_id_].addObservation(fpred, centroid, timestamp);
    }
   
    // -- If a corresponding image exists, draw in it.
    ostringstream oss;
    oss << "image" << fixed << setprecision(16) << setw(16) << setfill('0') << timestamp << ".jpg";
    string filename = oss.str();
    string path = images_dir + "/" + filename;
    if(!bfs::exists(path))
      continue;

    cv::Mat3b img = cv::imread(path);
    for(size_t j = 0; j < dets.size(); ++j) {
      const Det& det = dets[j];

      NameMapping cmap(det.class_names_);
      Label fpred(det.frame_predictions_);
      Label tpred = filters[det.track_id_].trackPrediction();
      float logodds = tpred(cmap.toId(show));
      if(opts.count("frame"))
        logodds = fpred(cmap.toId(show));

      // cout << det.track_id_ << " " << tpred(cmap.toId(show)) << " " << fpred(cmap.toId(show)) << " "
      //      << setprecision(16) << setw(16) << setfill('0') << timestamp << endl;
      
      cv::Point ul(max(0, det.ulx_ - pad), max(0, det.uly_ - pad));
      cv::Point lr(min(det.lrx_ + pad, img.cols), min(det.lry_ + pad, img.rows));
      cv::Scalar color(127, 127, 127);
      if(logodds > 0) {
        float val = 255 * logistic(logodds, sigma);
        //color = cv::Scalar(127 - val, 127 - val, 127 + 127 * val);  // red
        color = cv::Scalar(127 - val, 127 + 127 * val, 127 - val);  // green
      }
      
      if(!opts.count("only-pos") || (opts.count("only-pos") && logodds > 0))
        cv::rectangle(img, ul, lr, color, 2);
    }
    
    // Save to the tmp dir.
    cv::imshow("img", img);
    cv::waitKey(2);
    cv::imwrite(tmpdir + "/" + filename, img);
  }

  // -- Generate the video.
  ostringstream oss;
  oss << "mencoder mf://" << tmpdir << "/*.jpg -mf fps="
      << fps << " -ovc x264 -x264encopts crf=13 -vf "
      << rotation << " -o detections.avi";
  int retval = system(oss.str().c_str());
  ROS_ASSERT(retval == 0);

  bfs::remove_all(tmpdir);
  return 0;
}
