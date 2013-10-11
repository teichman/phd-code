#include <algorithm>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <ros/assert.h>

using namespace std;
namespace bpo = boost::program_options;
namespace bfs = boost::filesystem;


cv::Mat3b blend(cv::Mat3b first, cv::Mat3b second, double weight)
{
  cv::Mat3b img(first.size());
  for(int y = 0; y < img.rows; ++y) {
    for(int x = 0; x < img.cols; ++x) {
      for(int i = 0; i < 3; ++i) { 
        img(y, x)[i] = (double)first(y, x)[i] * weight + (double)(second(y, x)[i] * (1 - weight));
      }
    }
  }
  return img;
}


inline bool isAlpha(char c)
{
  return isalpha(c);
}

bool isVideoPath(const string& str)
{
  return (find_if(str.begin(), str.end(), isAlpha) != str.end());
}

void extractClips(const bpo::variables_map& opts,
                  const std::string& video_path,
                  const std::vector<double>& crop_times,
                  std::vector< std::vector<cv::Mat3b> >* clips)
{
  cv::VideoCapture cap(video_path);
  if(!cap.isOpened()) {
    cout << "Failed to open " << video_path << endl;
    return;
  }

  // -- Get all the images we want for the clips in this video.
  double fps = cap.get(CV_CAP_PROP_FPS);
  cv::Mat3b orig;
  int num = 0;
  double time = 0;
  int orig_num_clips = clips->size();
  clips->resize(clips->size() + crop_times.size() / 2);
  
  while(true) {
    cap >> orig;
    time = num / fps;
    ++num;

    if(time >= crop_times.back())
      break;

    int clipnum = -1;
    for(size_t i = 1; i < crop_times.size(); i+=2)
      if(time >= crop_times[i-1] && time < crop_times[i])
        clipnum = orig_num_clips + i / 2;

    if(clipnum != -1) {
      cout << time << endl;
      (*clips)[clipnum].push_back(orig.clone());
    }
  }
}

void mergeClips(const bpo::variables_map& opts,
                const std::vector< std::vector<cv::Mat3b> >& clips)
{
  // -- Blend frames.
  vector<cv::Mat3b> imgs;
  size_t overlap = opts["overlap"].as<size_t>();
  for(size_t i = 0; i < clips.size(); ++i) {
    cout << clips[i].size() << endl;
    ROS_ASSERT(clips[i].size() > overlap);
  }

  // -- Add the first clip, without the tail.
  for(size_t i = 0; i < clips[0].size() - overlap; ++i) {
    imgs.push_back(clips[0][i].clone());
  }

  // -- Process all remaining clips.
  for(size_t i = 0; i < clips.size(); ++i) {
    // If this is the last clip, add the tail in without a fade.
    if(i == clips.size() - 1) {
      for(size_t j = clips[i].size() - overlap; j < clips[i].size(); ++j) {
        imgs.push_back(clips[i][j].clone());
      }
    }

    // If there's another one, fade it in and add the middle.
    else {
      for(size_t j = clips[i].size() - overlap; j < clips[i].size(); ++j) {
        double weight = 1.0 - ((double)j - clips[i].size() + overlap) / (double)overlap;
        cout << "weight: " << weight << endl;
        cv::Mat3b img = blend(clips[i][j], clips[i+1][j - clips[i].size() + overlap], weight);
        imgs.push_back(img);
      }

      for(size_t j = overlap; j < clips[i+1].size() - overlap; ++j) {
        imgs.push_back(clips[i+1][j].clone());
      }
    }
  }

  // -- Save the output.
  string output_dir = opts["output-dir"].as<string>();
  if(!bfs::exists(output_dir))
    bfs::create_directory(output_dir);
  for(size_t i = 0; i < imgs.size(); ++i) {
    ostringstream oss;
    oss << output_dir << "/frame" << setw(6) << setfill('0') << i << ".png";
    cv::imwrite(oss.str(), imgs[i]);
  }
}

int main(int argc, char** argv)
{
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  vector<string> args;
  size_t overlap;
  string output_dir;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("args", bpo::value(&args)->required()->multitoken(), "")
    ("overlap", bpo::value(&overlap)->default_value(30), "frames of overlap")
    ("output-dir,o", bpo::value(&output_dir)->required())
    ;

  p.add("args", -1);
  
  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << argv[0] << " [OPTS] VIDEO START STOP [ START STOP ... ]" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  vector<string> video_paths;
  vector< vector<double> > crop_times;
  for(size_t i = 0; i < args.size(); ++i) {
    if(isVideoPath(args[i])) {
      video_paths.push_back(args[i]);
      crop_times.push_back(vector<double>());
    }
    else {
      ROS_ASSERT(!crop_times.empty());
      crop_times.back().push_back(atof(args[i].c_str()));
    }
  }

  ROS_ASSERT(video_paths.size() == crop_times.size());
  for(size_t i = 0; i < video_paths.size(); ++i) {
    cout << "Video: " << video_paths[i] << endl;
    for(size_t j = 0; j < crop_times[i].size(); ++j) {
      cout << "  " << crop_times[i][j] << endl;
    }
  }

  // -- Dump the frames.
  vector< vector<cv::Mat3b> > clips;
  for(size_t i = 0; i < video_paths.size(); ++i) {
    cout << "Working on " << video_paths[i] << endl;
    extractClips(opts, video_paths[i], crop_times[i], &clips);
  }

  // -- Blend the imagery in RAM.
  mergeClips(opts, clips);
  
  // -- Compile the video and save to disk.
  ostringstream oss;
  oss << "mencoder mf://" << output_dir
      << "/*.png -mf fps=30 -ovc x264 -x264encopts crf=13 -o "
      << output_dir << "/cropped.avi";
  int retval = system(oss.str().c_str());
  ROS_ASSERT(retval == 0);

  oss.str("");
  oss << "rm " << output_dir << "/*.png";
  retval = system(oss.str().c_str());
  ROS_ASSERT(retval == 0);
  
  return 0;
}
