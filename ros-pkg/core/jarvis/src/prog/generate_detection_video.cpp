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

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  string images_dir;
  string detection_metadata_dir;
  string show;
  int pad;
  double sigma;
  double fps;
  string rotation;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("images-dir", bpo::value(&images_dir)->required(), "")
    ("detection-metadata-dir", bpo::value(&detection_metadata_dir)->required(), "")
    ("show", bpo::value(&show)->required(), "class to show detections for")
    ("pad", bpo::value(&pad)->default_value(7), "padding on the bounding box")
    ("sigma", bpo::value(&sigma)->default_value(5), "in the logistic function for coloring")
    ("fps", bpo::value(&fps)->default_value(30), "")
    ("rotation", bpo::value(&rotation)->default_value("rotate=1,rotate=1"), "mencoder rotation option")
    ("only-pos", "Show only positive detections rather than everything")
    ("frame", "Show frame predictions rather than track predictions")
    ;

  p.add("images-dir", 1);
  p.add("detection-metadata-dir", 1);
  p.add("show", 1);
  
  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << argv[0] << " [OPTS] IMAGES_DIR DETECTION_METADATA_DIR SHOW" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  string tmpdir = ".generate_detection_video-tmpdir";
  if(bfs::exists(tmpdir))
    bfs::remove_all(tmpdir);
  bfs::create_directory(tmpdir);

  std::map<size_t, DiscreteBayesFilter> filters;
  
  vector<string> image_paths = glob(images_dir + "/*.jpg");
  for(size_t i = 0; i < image_paths.size(); ++i) {
    cv::Mat3b img = cv::imread(image_paths[i]);

    // Get the corresponding detection file.
    bfs::path path(image_paths[i]);
    string leaf = path.leaf().string();
    string timestamp_string = leaf.substr(5, leaf.size() - 9);
    string detection_filename = "detection" + timestamp_string + ".txt";
    string detection_path = detection_metadata_dir + "/" + detection_filename;

    // If it exists, draw in the image.
    if(bfs::exists(detection_path)) {
      cout << detection_path << endl;

      ifstream file(detection_path.c_str());
      while(true) {
        string line;
        getline(file, line);
        if(file.eof())
          break;
        istringstream iss(line);
        size_t track_id;
        iss >> track_id;
        int ulx, uly, lrx, lry;
        iss >> ulx;
        iss >> uly;
        iss >> lrx;
        iss >> lry;
        vector<float> frame_prediction_stl;
        vector<string> class_names;
        while(!iss.eof()) {
          string class_name;
          iss >> class_name;
          float logodds;
          iss >> logodds;
          frame_prediction_stl.push_back(logodds);
          class_names.push_back(class_name);
        }

        // -- Update DiscreteBayesFilter.  Use fake centroid motion because we don't have
        //    that info anymore.
        Label fpred(frame_prediction_stl);
        double timestamp = atof(timestamp_string.c_str());
        VectorXf centroid = VectorXf::Ones(3); 
        centroid *= timestamp;
        filters[track_id].addObservation(fpred, centroid, timestamp);
        NameMapping cmap(class_names);
        
        // -- Draw.
        Label tpred = filters[track_id].trackPrediction();
        float logodds = tpred(cmap.toId(show));
        if(opts.count("frame"))
          logodds = fpred(cmap.toId(show));
        cv::Point ul(max(0, ulx - pad), max(0, uly - pad));
        cv::Point lr(min(lrx + pad, img.cols), min(lry + pad, img.rows));
        cv::Scalar color(127, 127, 127);
        if(logodds > 0) {
          float val = 255 * logistic(logodds, sigma);
          //color = cv::Scalar(127 - val, 127 - val, 127 + 127 * val);  // red
          color = cv::Scalar(127 - val, 127 + 127 * val, 127 - val);  // green
        }

        if(!opts.count("only-pos") || (opts.count("only-pos") && logodds > 0))
          cv::rectangle(img, ul, lr, color, 2);
      }
    }

    // Save to the tmp dir.
    cv::imshow("img", img);
    cv::waitKey(2);
    cv::imwrite(tmpdir + "/" + leaf, img);
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
