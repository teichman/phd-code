#include <image_labeler/opencv_view.h>
#include <openni2_interface/openni2_interface.h>
#include <openni2_interface/openni_helpers.h>
#include <agent/agent.h>
#include <boost/program_options.hpp>

using namespace std;

//! Rigid Object Detector
class RodVisualizer : public OpenNI2Handler, public OpenCVViewDelegate, public Agent
{
public:
  RodVisualizer();
  
protected:
  OpenNI2Interface oni_;
  OpenCVView view_;
  std::vector<cv::Point2i> selected_points_;
  
  // color and depth contain sensor timestamps.
  // timestamp contains the wall time, in seconds.
  void rgbdCallback(openni::VideoFrameRef color,
                    openni::VideoFrameRef depth,
                    size_t frame_id, double timestamp);
  void _run();
  void mouseEvent(int event, int x, int y, int flags, void* param);
  void keypress(char c);
};

RodVisualizer::RodVisualizer() :
  oni_(OpenNI2Interface::VGA, OpenNI2Interface::VGA),
  view_("Rigid Object Detector")
{
  view_.setDelegate(this);
}

void RodVisualizer::rgbdCallback(openni::VideoFrameRef color,
                                 openni::VideoFrameRef depth,
                                 size_t frame_id, double timestamp)
{
  cv::Mat3b vis = oniToCV(color);
  view_.updateImage(vis);
  char c = view_.cvWaitKey(2);
  if(c != -1)
    keypress(c);
}

void RodVisualizer::mouseEvent(int event, int x, int y, int flags, void* param)
{
  if(event == CV_EVENT_LBUTTONUP) {
    selected_points_.push_back(cv::Point2i(x, y));
    cout << "Got point " << selected_points_.back() << endl;
  }
}

void RodVisualizer::_run()
{
  oni_.setHandler(this);
  oni_.run();
}

void RodVisualizer::keypress(char c)
{
  cout << "Got keypress: " << c << endl;
  
  switch(c) {
  case 'q':
    oni_.stop();
    break;
  default:
    break;
  }
}


int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  opts_desc.add_options()
    ("help,h", "produce help message")
    ;

  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << argv[0] << " [OPTS]" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  RodVisualizer rodvis;
  rodvis.run();
  
  return 0;
}
