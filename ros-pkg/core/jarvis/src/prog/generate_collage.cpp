#include <online_learning/collage_generator.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/program_options.hpp>
#include <jarvis/tracker.h>

using namespace std;
using namespace Eigen;
namespace bfs = boost::filesystem;

class Collagen : public TrackDatasetCollageGenerator
{
public:
  size_t boxsize_;
  
  Collagen(size_t boxsize, TrackDataset::ConstPtr td, const std::string& name = "") :
    TrackDatasetCollageGenerator(td, name),
    boxsize_(boxsize)
  {
  }

  cv::Mat3b visualize(const Instance& inst)
  {
    const Blob& blob = *boost::any_cast<Blob::Ptr>(inst.raw_);
    cv::Mat3b img = blob.image();
    // TODO:  Don't hardcode this.
    cv::flip(img, img, -1);  // Flip both x and y.

    // Scale it down.
    cv::Mat3b scaled;
    cv::resize(img, scaled, cv::Size(img.cols / 2, img.rows / 2), cv::INTER_NEAREST);
    return scaled;
  }
};

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  int size;
  vector<string> td_paths;
  size_t num_frames;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("size,s", bpo::value(&size)->default_value(5), "Rows and columns in the collage (number of images)")
    ("tds,d", bpo::value(&td_paths)->required()->multitoken(), "")
    ("image,i", "")
    ("video,v", "")
    ("num-frames,n", bpo::value(&num_frames)->default_value(0), "If writing a video, use this number of frames.  0 => use max track length.")
    ;

  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << argv[0] << " [OPTS] --tds TD [ TD ... ]" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  srand(time(NULL));

  if(!opts.count("image") && !opts.count("video")) {
    cout << "You must set --image or --video to get any output." << endl;
    return 0;
  }

  for(size_t i = 0; i < td_paths.size(); ++i) {
    string td_path = td_paths[i];
  
    TrackDataset::Ptr td(new TrackDataset);
    cout << "Loading " << td_path << endl;
    td->load(td_path);
    if(td->size() == 0) {
      cout << "TrackDataset is empty." << endl;
      continue;
    }
    
    Collagen collagen(size, td);
    
    // -- Image output.
    if(opts.count("image")) {
      string image_path;
      bfs::path p(td_path);
      if(p.has_parent_path())
        image_path = p.parent_path().string() + "/" + p.stem().string() + ".png";
      else
        image_path = p.stem().string() + ".png";
      cout << "Saving image to " << image_path << endl;
      cv::Mat3b img = collagen.generateGrid(size, size);
      cv::imwrite(image_path, img);
    }
    
    // -- Video output.
    if(opts.count("video")) {
      string video_path;
      bfs::path p(td_path);
      if(p.has_parent_path())
        video_path = p.parent_path().string() + "/" + p.stem().string() + ".avi";
      else
        video_path = p.stem().string() + ".avi";
      cout << "Saving video to " << video_path << endl;
      collagen.writeVideo(size, size, video_path, 60, num_frames);
    }
  }
  
  return 0;
}

