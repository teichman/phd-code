#include <boost/program_options.hpp>
#include <asp/aspvis.h>
#include <asp/simple_segmentation_pipeline.h>

using namespace asp::example;

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  opts_desc.add_options()
    ("help,h", "produce help message")
    ("img", bpo::value<string>()->required(), "Image to segment")
    ("scale", bpo::value<double>()->default_value(1.0), "Scale factor to apply when visualizing")
    ;

  p.add("img", 1);

  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: simple_segmenter [OPTS] IMG" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  cv::Mat3b img = cv::imread(opts["img"].as<string>());
  Asp asp(1);
  generateSimpleSegmentationPipeline(&asp);
  AspVis vis(&asp, img, opts["scale"].as<double>());
  vis.run();

  return 0;
}
