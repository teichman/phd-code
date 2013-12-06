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
  string reticle_path;
  float alpha;
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
    ("reticle", bpo::value(&reticle_path), "Image to use instead of a box.  Must be white on black.")
    ("alpha", bpo::value(&alpha)->default_value(0.1), "How fast the reticle should follow the cat around")
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

  // -- Set up reticle.
  cv::Mat1b reticle;
  if(opts.count("reticle")) {
    cout << "Using reticle at " << reticle_path << endl;
    reticle = cv::imread(reticle_path, CV_LOAD_IMAGE_GRAYSCALE);
  }
  cv::Point2f retpt(160, 120);
  int reticle_target_id = -1;
  int reticle_counter = 0;
     
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

    // -- Reticle mode.
    if(reticle.rows > 0) {
      // -- Get most likely target.
      int best_target_id = -1;
      float max_logodds = 0;
      for(size_t j = 0; j < dets.size(); ++j) {
        const Det& det = dets[j];
        NameMapping cmap(det.class_names_);
        Label tpred = filters[det.track_id_].trackPrediction();
        float logodds = tpred(cmap.toId(show));
        if(logodds > max_logodds) {
          max_logodds = logodds;
          best_target_id = dets[j].track_id_;
        }
      }

      // Check if the current target exists.  If so, get its logodds.
      bool current_target_exists = false;
      float current_target_logodds;
      for(size_t j = 0; j < dets.size(); ++j) {
        if(reticle_target_id == (int)dets[j].track_id_) {
          const Det& det = dets[j];
          NameMapping cmap(det.class_names_);
          Label tpred = filters[det.track_id_].trackPrediction();
          current_target_exists = true;            
          current_target_logodds = tpred(cmap.toId(show));
        }
      }

      // If current target doesn't exist, take the most likely target if any.
      if(!current_target_exists) {
        if(best_target_id >= 0) {
          reticle_target_id = best_target_id;
          current_target_exists = true;
          reticle_counter = 0;
        }
      }
      // Otherwise, if there is a better one and we haven't switched in a while, switch to it.
      else if(max_logodds > current_target_logodds && reticle_counter > 300) {
        reticle_target_id = best_target_id;
        reticle_counter = 0;
      }
      ++reticle_counter;
      
      // -- If we have a target, move towards it.
      if(current_target_exists) {
        for(size_t j = 0; j < dets.size(); ++j) {
          const Det& det = dets[j];
          if((int)det.track_id_ == reticle_target_id) {
            cv::Point2f center((det.lrx_ + det.ulx_) / 2, (det.lry_ + det.uly_) / 2);
            retpt = alpha * center + (1.0 - alpha) * retpt;
            break;
          }
        }
      }

      // -- Draw the reticle.
      cv::Vec3b color = cv::Vec3b(0, 0, 255);
      for(int y = 0; y < reticle.rows; ++y) {
        for(int x = 0; x < reticle.cols; ++x) {
          cv::Point drawpt;
          drawpt.y = retpt.y - reticle.rows / 2 + y;
          drawpt.x = retpt.x - reticle.cols / 2 + x;
          if(drawpt.y < 0 || drawpt.y >= img.rows)
            continue;
          if(drawpt.x < 0 || drawpt.x >= img.cols)
            continue;

          img(drawpt) = ((float)reticle(y, x)) / 255.0 * color + (1 - (float)reticle(y, x) / 255.0) * img(drawpt);
        }
      }
    }

    // -- Box mode
    else {
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
        
        cv::Vec3b color(127, 127, 127);
        if(logodds > 0) {
          float val = 255 * logistic(logodds, sigma);
          color = cv::Vec3b(127 - val, 127 + 127 * val, 127 - val);  // green
        }
      

        else {
          cv::Point ul(max(0, det.ulx_ - pad), max(0, det.uly_ - pad));
          cv::Point lr(min(det.lrx_ + pad, img.cols), min(det.lry_ + pad, img.rows));
          if(!opts.count("only-pos") || (opts.count("only-pos") && logodds > 0))
            cv::rectangle(img, ul, lr, cv::Scalar(color[0], color[1], color[2]), 2);
        }
      }
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
