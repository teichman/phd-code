#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <bag_of_tricks/glob.h>
#include <jarvis/compression_helpers.h>
#include <fstream>

using namespace std;
namespace bfs = boost::filesystem;

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  string rgb_dir;
  string path;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("rgb-dir", bpo::value(&rgb_dir)->required(), "Directory of RGB images")
    ("output,o", bpo::value(&path)->required(), "Output path")
    ;

  p.add("rgb-dir", 1);
  p.add("output", 1);

  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << argv[0] << " [OPTS] RGB_DIR OUTPUT_PATH" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }


  // -- Load the data.
  vector<string> paths = glob(rgb_dir + "/*.png");
  vector<cv::Mat3b> color;
  color.reserve(paths.size());
  for(size_t i = 0; i < paths.size(); ++i) {
    color.push_back(cv::imread(paths[i]));
    cv::imshow("Color", color.back());
    cv::waitKey(2);
  }
  cout << "Loaded " << color.size() << " test images." << endl;

  // -- Encode.
  // H264Encoder enc(25, 2e5);
  // enc.initialize(color[0].cols, color[0].rows);
  // for(size_t i = 0; i < color.size(); ++i)
  //   enc.addFrame(color[i]);

  // // -- Write to disk.
  // enc.finalize();
  // enc.save(path);
  // cout << "Wrote to " << path << endl;
  
  EncodingOptions encopts;
  encopts.fps_ = 30;
  encopts.crf_ = 13;

  vector<uint8_t> blob;
  encodeH264Shm(encopts, color, &blob);

  ofstream file(path.c_str());
  file.write((char*)blob.data(), blob.size());
  file.close();

  return 0;
}
