#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <rgbd_sequence/stream_sequence.h>
#include <xpl_calibration/discrete_depth_distortion_model.h>

using namespace std;
using namespace Eigen;
using namespace rgbd;

bool process(const StreamSequence& sseq, size_t idx, DiscreteDepthDistortionModel* dddm,
	     double* mean_range, double* error, double* rms)
{
  Frame frame;
  sseq.readFrame(idx, &frame);
  if(dddm)
    dddm->undistort(&frame);
  Cloud cloud;
  sseq.model_.frameToCloud(frame, &cloud);

  double total = 0;
  for(size_t i = 0; i < cloud.size(); ++i)
    if(isFinite(cloud[i]))
      ++total;

  if(total < 30000)
    return false;

  Vector3f mean = Vector3f::Zero();
  for(size_t i = 0; i < cloud.size(); ++i)
    if(isFinite(cloud[i]))
      mean += cloud[i].getVector3fMap();
  mean /= total;
  
  Matrix3d X = Matrix3d::Zero();
  for(size_t i = 0; i < cloud.size(); ++i)
    if(isFinite(cloud[i]))
      X += ((cloud[i].getVector3fMap() - mean) * (cloud[i].getVector3fMap() - mean).transpose()).cast<double>();

  JacobiSVD<Matrix3d> svd(X, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Matrix3d V = svd.matrixV();
  Vector3f normal = V.col(2).cast<float>();
  
  double mean_offset = 0;
  for(size_t i = 0; i < cloud.size(); ++i)
    if(isFinite(cloud[i]))
      mean_offset += normal.dot(cloud[i].getVector3fMap());
  mean_offset /= total;

  *mean_range = 0;
  for(size_t i = 0; i < cloud.size(); ++i)
    if(isFinite(cloud[i]))
      *mean_range += cloud[i].z;
  *mean_range /= total;
  
  *error = 0;
  for(size_t i = 0; i < cloud.size(); ++i)
    if(isFinite(cloud[i]))
      *error += fabs(normal.dot(cloud[i].getVector3fMap()) - mean_offset);
  *error /= total;

  *rms = 0;
  for(size_t i = 0; i < cloud.size(); ++i)
    if(isFinite(cloud[i]))
      *rms += pow(normal.dot(cloud[i].getVector3fMap()) - mean_offset, 2);
  *rms /= total;
  *rms = sqrt(*rms);
  
  return true;
}

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  namespace bfs = boost::filesystem;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  opts_desc.add_options()
    ("help,h", "produce help message")
    ("sseq", bpo::value<string>()->required(), "StreamSequence, i.e. asus data.")
    ("intrinsics", bpo::value<string>())
    ;

  p.add("sseq", 1);
  
  bpo::variables_map opts;
  bool badargs = false;
  try {
    bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
    bpo::notify(opts);
  }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << bfs::basename(argv[0]) << " SSEQ" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  StreamSequence sseq;
  sseq.load(opts["sseq"].as<string>());

  DiscreteDepthDistortionModel* dddm = NULL;
  if(opts.count("intrinsics")) {
    dddm = new DiscreteDepthDistortionModel;
    dddm->load(opts["intrinsics"].as<string>());
  }
  
  for(size_t i = 10; i < sseq.size(); i += 10) {
    double mean_range;
    double error;
    double rms;
    bool valid = process(sseq, i, dddm, &mean_range, &error, &rms);
    if(valid)
      cout << mean_range << " " << error << " " << rms << endl;
  }

  return 0;
}
