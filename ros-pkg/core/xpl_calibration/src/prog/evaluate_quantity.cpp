#include <boost/program_options.hpp>
#include <xpl_calibration/slam_calibrator.h>
#include <xpl_calibration/primesense_slam.h>

using namespace std;
using namespace Eigen;
using namespace rgbd;
namespace bpo = boost::program_options;
namespace bfs = boost::filesystem;

void computeDistortion(const Frame& frame, const Frame& mapframe,
		       double* total_error, double* num_pts)
{
  ROS_ASSERT(frame.depth_->cols() == mapframe.depth_->cols());
  ROS_ASSERT(frame.depth_->rows() == mapframe.depth_->rows());

  double total_error_local = 0;
  double num_pts_local = 0;
  for(int x = 0; x < frame.depth_->cols(); ++x) {
    for(int y = 0; y < frame.depth_->rows(); ++y) {

      if(frame.depth_->coeffRef(y, x) == 0)
	continue;
      if(mapframe.depth_->coeffRef(y, x) == 0)
	continue;

      double meas = frame.depth_->coeffRef(y, x) * 0.001;
      double gt = mapframe.depth_->coeffRef(y, x) * 0.001;
      double mult = gt / meas;
      if(mult > MAX_MULT || mult < MIN_MULT)
	continue;

      total_error_local += fabs(meas - gt);
      ++num_pts_local;
    }
  }

  static boost::shared_mutex shared_mutex;
  boost::unique_lock<boost::shared_mutex> lockable_unique_lock(shared_mutex);
  *total_error += total_error_local;
  *num_pts += num_pts_local;
}

void evaluate(const bpo::variables_map& opts,
	      const DiscreteDepthDistortionModel& intrinsics,
	      const Cloud& map,
	      const StreamSequence& sseq,
	      const Trajectory& traj,
	      double* raw_total_error, double* raw_num_pts,
	      double* undistorted_total_error, double* undistorted_num_pts)
{
  // -- For all poses, compute the distortion with and without the intrinsics.
  #pragma omp parallel for
  for(size_t i = 0; i < traj.size(); ++i) {
    if(!traj.exists(i))
      continue;

    Frame frame;
    sseq.readFrame(i, &frame);
    Affine3f transform = traj.get(i).inverse().cast<float>();
    Cloud transformed;
    pcl::transformPointCloud(map, transformed, transform);
    Frame mapframe;
    sseq.model_.cloudToFrame(transformed, &mapframe);

    // cv::imshow("mapframe", mapframe.depthImage());
    // cv::imshow("frame", frame.depthImage());
    // cv::waitKey(2);

    computeDistortion(frame, mapframe, raw_total_error, raw_num_pts);
    intrinsics.undistort(&frame);
    computeDistortion(frame, mapframe, undistorted_total_error, undistorted_num_pts);
  }
}

void evaluate(const bpo::variables_map& opts,
	      const DiscreteDepthDistortionModel& intrinsics,
	      const vector<Cloud>& maps,
	      const vector<StreamSequence::ConstPtr>& sseqs,
	      const vector<Trajectory>& trajectories,
	      const string& eval_path)
{
  ROS_ASSERT(bfs::exists(eval_path));
  intrinsics.save(eval_path + "/intrinsics");

  // -- Test undistortion time.
  double count = 0;
  HighResTimer hrt;
  Frame frame;
  for(size_t i = 0; i < min((size_t)500, sseqs[0]->size()); i += 10) {
    sseqs[0]->readFrame(i, &frame);
    hrt.start();
    intrinsics.undistort(&frame);
    hrt.stop();
    ++count;
  }
  double mean_undistortion_time_ms = hrt.getMilliseconds() / count;
  cout << "Evaluated undistortion time for " << count << " frames." << endl;
  cout << "Mean undistortion time (ms): " << mean_undistortion_time_ms << endl;
  
  // -- Run quantitative evaluation on all maps.
  double raw_total_error = 0;
  double raw_num_pts = 0;
  double undistorted_total_error = 0;
  double undistorted_num_pts = 0;
  for(size_t i = 0; i < sseqs.size(); ++i) {
    evaluate(opts, intrinsics, maps[i], *sseqs[i], trajectories[i],
	     &raw_total_error, &raw_num_pts,
	     &undistorted_total_error, &undistorted_num_pts);
  }
  double raw_mean_error = raw_total_error / raw_num_pts;
  double undistorted_mean_error = undistorted_total_error / undistorted_num_pts;
  
  // -- Save output.
  ofstream file;
  file.open((eval_path + "/results.txt").c_str());
  ROS_ASSERT(file.is_open());
  file << "Mean undistortion time (ms): " << mean_undistortion_time_ms << endl;
  file << "Raw total error: " << raw_total_error << endl;
  file << "Raw num_pts: " << raw_num_pts << endl;
  file << "Raw mean error: " << raw_mean_error << endl;
  file << "Undistorted total error: " << undistorted_total_error << endl;
  file << "Undistorted num_pts: " << undistorted_num_pts << endl;
  file << "Undistorted mean error: " << undistorted_mean_error << endl;
  file << "Error reduction: " << (raw_mean_error - undistorted_mean_error) / raw_mean_error << endl;
  file.close();
}

DiscreteDepthDistortionModel calibrate(const bpo::variables_map& opts,
				       const vector<StreamSequence::ConstPtr>& sseqs,
				       const vector<Trajectory>& trajectories,
				       int skip_idx = -1)
{
  ROS_ASSERT(sseqs.size() == trajectories.size());
  
  vector<StreamSequence::ConstPtr> sseq_train;
  vector<Trajectory> traj_train;
  for(size_t i = 0; i < sseqs.size(); ++i) {
    if((int)i == skip_idx)
      continue;
    sseq_train.push_back(sseqs[i]);
    traj_train.push_back(trajectories[i]);
  }

  SlamCalibrator calibrator(sseq_train[0]->model_, MAX_RANGE_MAP, opts["vgsize"].as<double>());
  calibrator.trajectories_ = traj_train;
  calibrator.sseqs_ = sseq_train;
  return calibrator.calibrateDiscrete();
}

void load(const vector<string>& sseq_paths, const vector<string>& traj_paths,
	  vector<StreamSequence::ConstPtr>* sseqs,
	  vector<Trajectory>* trajectories,
	  vector<string>* names)
{
  ROS_ASSERT(sseq_paths.size() == traj_paths.size());
  for(size_t i = 0; i < sseq_paths.size(); ++i) {
    cout << endl;
    string path = sseq_paths[i];
    cout << "Loading StreamSequence at " << path << endl;
    StreamSequence::Ptr sseq(new StreamSequence);
    sseq->load(path);
    sseqs->push_back(sseq);
      
    // Get the StreamSequence name.
    if(path[path.size() - 1] == '/')
      path = path.substr(0, path.size() - 1);
    string name = path.substr(path.find_last_of("/") + 1, string::npos);
    cout << "StreamSequence name: " << name << endl;
    names->push_back(name);

    cout << "Loading Trajectory at " << traj_paths[i] << endl;
    Trajectory traj;
    traj.load(traj_paths[i]);
    trajectories->push_back(traj);
  }
}

int main(int argc, char** argv)
{
  vector<string> sseq_paths_train;
  vector<string> traj_paths_train;
  vector<string> sseq_paths_test;
  vector<string> traj_paths_test;
  string output_path;

  namespace bfs = boost::filesystem;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  opts_desc.add_options()
    ("help,h", "produce help message")
    ("sseqs-train", bpo::value< vector<string> >(&sseq_paths_train)->required()->multitoken(), "StreamSequences.")
    ("trajs-train", bpo::value< vector<string> >(&traj_paths_train)->required()->multitoken(), "Trajectories.")
    ("sseqs-test", bpo::value< vector<string> >(&sseq_paths_test)->required()->multitoken(), "StreamSequences.")
    ("trajs-test", bpo::value< vector<string> >(&traj_paths_test)->required()->multitoken(), "Trajectories.")
    ("output", bpo::value<string>(&output_path)->required(), "Directory to put results.  Must not exist; will be created.")
    ("vgsize", bpo::value<double>()->default_value(0.01), "Size of voxel grid cells.")
    ;
     
  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << bfs::basename(argv[0]) << " [ OPTS ] --sseqs-train SSEQ [ SSEQ ... ] --trajs-train TRAJ [ TRAJ ... ] --sseqs-test SSEQ [ SSEQ ... ] --trajs-test TRAJ [ TRAJ ... ] --output OUTPUT" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  cout << endl;
  cout << "--------------------" << endl;
  ROS_ASSERT(!bfs::exists(output_path));
  cout << "Saving output to " << output_path << endl;
  cout << "--------------------" << endl;

  vector<StreamSequence::ConstPtr> sseqs_train;
  vector<Trajectory> trajectories_train;
  vector<string> names_train;
  load(sseq_paths_train, traj_paths_train, &sseqs_train, &trajectories_train, &names_train);
  vector<StreamSequence::ConstPtr> sseqs_test;
  vector<Trajectory> trajectories_test;
  vector<string> names_test;
  load(sseq_paths_test, traj_paths_test, &sseqs_test, &trajectories_test, &names_test);

  // -- Build the test maps once.
  cout << "Building test maps." << endl;
  vector<Cloud> maps(sseqs_test.size());
  for(size_t i = 0; i < maps.size(); ++i)
    maps[i] = *SlamCalibrator::buildMap(*sseqs_test[i], trajectories_test[i], MAX_RANGE_MAP, opts["vgsize"].as<double>());
  cout << "Done." << endl;
  
  cout << endl;
  cout << "--------------------" << endl;
  cout << endl;

  bfs::create_directory(output_path);
  int num_orderings = 10;
  for(int i = 0; i < num_orderings; ++i) {
    ostringstream oss;
    oss << output_path << "/ordering" << setw(3) << setfill('0') << i;
    string ordering_path = oss.str();
    bfs::create_directory(ordering_path);
    
    // -- Get a random ordering of the training data.
    vector<int> indices(sseqs_train.size());
    for(size_t j = 0; j < indices.size(); ++j)
      indices[j] = j;
    random_shuffle(indices.begin(), indices.end());

    // -- Add datasets and evaluate in order.
    for(size_t num_datasets = 1; num_datasets <= indices.size(); ++num_datasets) {
      ostringstream oss;
      oss << ordering_path << "/" << setw(3) << setfill('0') << num_datasets;
      string eval_path = oss.str();
      bfs::create_directory(eval_path);

      // -- Set up the training subset.
      vector<StreamSequence::ConstPtr> sseqs_train_subset;
      vector<Trajectory> trajectories_train_subset;
      vector<string> names_train_subset;
      for(size_t n = 0; n < num_datasets; ++n) {
	size_t idx = indices[n];
	sseqs_train_subset.push_back(sseqs_train[idx]);
	trajectories_train_subset.push_back(trajectories_train[idx]);
	names_train_subset.push_back(names_train[idx]);
      }

      // -- Save what data we're operating on.
      VectorXd training_seconds(sseqs_train_subset.size());
      VectorXd training_frames(sseqs_train_subset.size());
      for(int j = 0; j < training_seconds.size(); ++j) {
	training_seconds(j) = sseqs_train_subset[j]->timestamps_.back() - sseqs_train_subset[j]->timestamps_.front();
	training_frames(j) = trajectories_train_subset[j].numValid();
      }
      VectorXd testing_seconds(sseqs_test.size());
      VectorXd testing_frames(sseqs_test.size());
      for(int j = 0; j < testing_seconds.size(); ++j) {
	testing_seconds(j) = sseqs_test[j]->timestamps_.back() - sseqs_test[j]->timestamps_.front();
	testing_frames(j) = trajectories_test[j].numValid();
      }

      ofstream f;
      f.open((eval_path + "/info.txt").c_str());
      f << "== Training set ==" << endl;
      f << "  <name> <num_frames> <seconds>" << endl;
      for(size_t j = 0; j < names_train_subset.size(); ++j)
	f << "  " << names_train_subset[j] << " " << training_frames(j) << " " << training_seconds(j) << endl;
      f << endl;
      f << "== Testing set ==" << endl;
      f << "  <name> <num_frames> <seconds>" << endl;
      for(size_t j = 0; j < names_test.size(); ++j)
	f << "  " << names_test[j] << " " << testing_frames(j) << " " << testing_seconds(j) << endl;
      f << endl;
      f << endl;
      f << "Total seconds of data used for training: " << training_seconds.sum() << endl;
      f << "Total number of frames used for training: " << training_frames.sum() << endl;
      f << "Total seconds of data used for testing: " << testing_seconds.sum() << endl;
      f << "Total number of frames used for testing: " << testing_frames.sum() << endl;

      // -- Train and evaluate.
      HighResTimer hrt;
      hrt.start();
      DiscreteDepthDistortionModel intrinsics = calibrate(opts, sseqs_train_subset, trajectories_train_subset);
      hrt.stop();
      f << "Calibration time (seconds): " << hrt.getSeconds() << endl;
      f.close();
	    
      evaluate(opts, intrinsics, maps, sseqs_test, trajectories_test, eval_path);
    }
  }
  
  return 0;
}
