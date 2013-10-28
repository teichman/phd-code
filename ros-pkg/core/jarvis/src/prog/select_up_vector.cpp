#include <boost/program_options.hpp>
#include <eigen_extensions/eigen_extensions.h>
#include <openni2_interface/openni2_interface.h>
#include <openni2_interface/openni_helpers.h>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace Eigen;

class UpSelector : public OpenNI2Handler
{
public:
  UpSelector();
  void rgbdCallback(openni::VideoFrameRef color, openni::VideoFrameRef depth,
                    size_t frame_id, double timestamp);
  void run();

protected:
  OpenNI2Interface oni_;
};

UpSelector::UpSelector() :
  oni_(OpenNI2Interface::VGA, OpenNI2Interface::VGA)
{
  oni_.setHandler(this);
}

void UpSelector::rgbdCallback(openni::VideoFrameRef color, openni::VideoFrameRef depth,
                              size_t frame_id, double timestamp)
{
  cv::Mat3b img = colorize(oniDepthToEigen(depth), 0.5, 7);
  cv::imshow("depth", img);
  char key = cv::waitKey(2);
  if(key == 'q')
    terminate();
}

void UpSelector::run()
{
  oni_.run();
}



int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  string output_path;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("output-path", bpo::value(&output_path)->default_value("up.eig.txt"), "")
    ;

  p.add("output-path", 1);
  
  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << argv[0] << " [OPTS] OUTPUT_PATH" << endl; 
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  cout << "Saving to " << output_path << endl;

  UpSelector ups;
  ups.run();
  
  return 0;
}
