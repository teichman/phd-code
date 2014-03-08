#include <boost/program_options.hpp>
#include <boost/foreach.hpp>
#include <pcl/common/centroid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <online_learning/dataset.h>
#include <online_learning/common.h>
#include <pcl/common/common.h>

using namespace std;
namespace bfs = boost::filesystem;
using namespace Eigen;

class GroupInductionVisualizer
{
public:
  //! In radians / frame.
  double rotation_speed_;
  //! Show this many frames per iter of group induction.
  size_t num_frames_per_iter_;
  //! For video encoding.
  double frames_per_second_;
  //! For vis_->spinOnce().
  double ms_per_frame_;
  //! in grid cells
  size_t induction_grid_width_;
  //! See proportionalSample.
  size_t induction_representation_;
  //! "overview" or "detail".
  std::string radius_method_;
  std::string output_dir_;
  int max_iter_;
  bool write_video_;
    
  GroupInductionVisualizer(const std::string& root,
                           const std::string& class_name,
                           double meters_per_cell);
  void run();
  ~GroupInductionVisualizer();
  
protected:
  std::string root_;
  std::string class_name_;
  //! iter number, path to .td of example inducted objects of class_name_.
  std::map<size_t, std::string> induction_paths_;
  //! iter number, paths to .tds of new user annotations.
  std::map<size_t, std::vector<std::string> > annotation_paths_;
  TrackDataset induction_td_;
  TrackDataset annotation_td_;
  std::vector<size_t> induction_indices_;
  std::vector<size_t> annotation_indices_;
  int iter_;
  double theta_;
  double meters_per_cell_;
  size_t frame_id_;
  int induction_viewport_id_;
  int annotation_viewport_id_;
  int separator_viewport_id_;
  double pos_z_multiplier_;
  double radius_;
  double radius_setpoint_;
  pcl::visualization::PCLVisualizer* vis_;
  
  void initialize();
  bool incrementIter();
  void loadAnnotations(const std::vector<std::string>& paths);
  void initializeIter();
  Cloud::Ptr generateGridContent(TrackDataset& td,
                                 const std::vector<size_t>& indices,
                                 size_t frame_id) const;
  Cloud::Ptr generateGridLines() const;
  //! Returns a number of indices into td.  Max num indices is num_slots.
  //! Each filled slot represents a certain number of tracks in the td.
  //! For example, if representation == 100 and td.size() == 330, you'll get three
  //! indices in the returned vector if num_slots is at least three, and num_slots
  //! otherwise.
  std::vector<size_t> proportionalSample(const TrackDataset& td,
                                         size_t num_slots,
                                         size_t representation,
                                         int td_size_override = -1) const;
  void setCameraPose();
  void getNumInductedTracks(int* num_pos, int* num_neg) const;
  void getNumAutoAnnotated(int* num_pos, int* num_neg) const;
  void trackIndexToGridElement(int idx, size_t* row, size_t* col) const;
  void keyboardCallback(const pcl::visualization::KeyboardEvent& event,
                                                 void* cookie);
};

GroupInductionVisualizer::GroupInductionVisualizer(const std::string& root,
                                                   const std::string& class_name,
                                                   double meters_per_cell) :
  rotation_speed_(0.2 * M_PI / 180.0),
  num_frames_per_iter_(120),
  frames_per_second_(20),
  ms_per_frame_(30),
  induction_grid_width_(5),
  induction_representation_(20),
  output_dir_("group_induction_video"),
  max_iter_(1e6),
  write_video_(false),
  root_(root),
  class_name_(class_name),
  iter_(-1),
  theta_(0),
  meters_per_cell_(meters_per_cell),
  frame_id_(0)
{
  initialize();
}

GroupInductionVisualizer::~GroupInductionVisualizer()
{
  if(vis_)
    delete vis_;
}

void GroupInductionVisualizer::getNumAutoAnnotated(int* num_pos, int* num_neg) const
{
  string tmpfile = ".giv-saotehustaohenu";

  {
    ostringstream oss;
    oss << "cat iter" << setw(5) << setfill('0') << iter_ << "/learner_status.txt | sed -n '/Auto-annotated training dataset/,/Unsupervised/p' | grep 'empty tracks' -A3 | grep " << class_name_ << " | awk '{print $2}' > " << tmpfile;
    int retval = system(oss.str().c_str());
    ROS_ASSERT(retval == 0);
    std::ifstream t(tmpfile.c_str());
    std::string str((std::istreambuf_iterator<char>(t)),
                    std::istreambuf_iterator<char>());
    *num_pos = atoi(str.c_str());
  }

  {
    ostringstream oss;
    oss << "cat iter" << setw(5) << setfill('0') << iter_ << "/learner_status.txt | sed -n '/Auto-annotated training dataset/,/Unsupervised/p' | grep 'empty tracks' -A3 | grep " << class_name_ << " | awk '{print $4}' > " << tmpfile;
    int retval = system(oss.str().c_str());
    ROS_ASSERT(retval == 0);
    std::ifstream t(tmpfile.c_str());
    std::string str((std::istreambuf_iterator<char>(t)),
                    std::istreambuf_iterator<char>());
    *num_neg = atoi(str.c_str());
  }

  bfs::remove(tmpfile);
}

void GroupInductionVisualizer::getNumInductedTracks(int* num_pos, int* num_neg) const
{
  string tmpfile = ".giv-aoeuaoeu";

  {
    ostringstream oss;
    oss << "cat iter" << setw(5) << setfill('0') << iter_ << "/learner_status.txt | sed -n '/Unsupervised dataset/,/Bookkeeping/p' | grep 'empty tracks' -A3 | grep " << class_name_ << " | awk '{print $2}' > " << tmpfile;
    int retval = system(oss.str().c_str());
    ROS_ASSERT(retval == 0);
    std::ifstream t(tmpfile.c_str());
    std::string str((std::istreambuf_iterator<char>(t)),
                    std::istreambuf_iterator<char>());
    *num_pos = atoi(str.c_str());
  }

  {
    ostringstream oss;
    oss << "cat iter" << setw(5) << setfill('0') << iter_ << "/learner_status.txt | sed -n '/Unsupervised dataset/,/Bookkeeping/p' | grep 'empty tracks' -A3 | grep " << class_name_ << " | awk '{print $4}' > " << tmpfile;
    int retval = system(oss.str().c_str());
    ROS_ASSERT(retval == 0);
    std::ifstream t(tmpfile.c_str());
    std::string str((std::istreambuf_iterator<char>(t)),
                    std::istreambuf_iterator<char>());
    *num_neg = atoi(str.c_str());
  }
  
  bfs::remove(tmpfile);
}

void GroupInductionVisualizer::setCameraPose()
{
  ROS_WARN("GroupInductionVisualizer::setCameraPose was broken by a PCL update.");

  // double grid_center = meters_per_cell_ * induction_grid_width_ / 2;
  // double pos_x = radius_ * cos(theta_) + grid_center;
  // double pos_y = radius_ * sin(theta_) + grid_center;
  // double pos_z = radius_ * pos_z_multiplier_ * .577;
  // double view_x = grid_center;
  // double view_y = grid_center;
  // double view_z = 0;
  // double up_x = 0;
  // double up_y = 0;
  // double up_z = 1;

  // vis_->setCameraPosition(pos_x, pos_y, pos_z,
  //                         view_x, view_y, view_z,
  //                         up_x, up_y, up_z,
  //                         induction_viewport_id_);
}

void GroupInductionVisualizer::keyboardCallback(const pcl::visualization::KeyboardEvent& event,
                                                void* cookie)
{
  if(!event.keyDown())
    return;
  
  char key = event.getKeyCode();
  switch(key) {
  case ' ':
    if(radius_method_ == "overview")
      radius_method_ = "detail";
    else
      radius_method_ = "overview";
    break;
  default:
    break;
  }
}

void GroupInductionVisualizer::initialize()
{
  // -- Visualizer.
  // Set the window size.  The rest doesn't matter as it will be reset.
  char* argv[] = {"aoeu", "-cam", "0,1000000/0,0,0/24.0342,-14.7529,22.264/-0.586992,0.219668,0.779222/0.523599/1777,1000/-5,-5"};
  int argc = 3;
  vis_ = new pcl::visualization::PCLVisualizer(argc, argv, "Group induction visualizer");
  vis_->registerKeyboardCallback(&GroupInductionVisualizer::keyboardCallback, *this);
  
  double half_sep_width = 0.001;  // out of 1.
  
  vis_->createViewPort(0, 0, 0.5 - half_sep_width, 1, induction_viewport_id_);
  vis_->addPointCloud(Cloud::Ptr(new Cloud), "induction", induction_viewport_id_);
  vis_->addPointCloud(Cloud::Ptr(new Cloud), "induction-grid", induction_viewport_id_);
  vis_->setBackgroundColor(0, 0, 0, induction_viewport_id_);
  vis_->addCoordinateSystem(meters_per_cell_, induction_viewport_id_);
  vis_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "induction", induction_viewport_id_);
  vis_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "induction-grid", induction_viewport_id_);

  vis_->createViewPort(0.5 + half_sep_width, 0, 1, 1, annotation_viewport_id_);
  vis_->addPointCloud(Cloud::Ptr(new Cloud), "annotation", annotation_viewport_id_);
  vis_->addPointCloud(Cloud::Ptr(new Cloud), "annotation-grid", annotation_viewport_id_);
  vis_->setBackgroundColor(0, 0, 0, annotation_viewport_id_);
  vis_->addCoordinateSystem(meters_per_cell_, annotation_viewport_id_);
  vis_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "annotation", annotation_viewport_id_);
  vis_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "annotation-grid", annotation_viewport_id_);

  setCameraPose();
  // This doesn't work.  Instead, we'll use another viewport.
  //vis_->setWindowBorders(true);
  vis_->createViewPort(0.5 - half_sep_width, 0, 0.5 + half_sep_width, 1, separator_viewport_id_);
  vis_->setBackgroundColor(1, 1, 1, separator_viewport_id_);

  // In PCL 1.7, but not the version we're using.  :/
  // Using the nasty argc / argv init instead.
  //vis_->setSize(1024, 768);  
  // Similarly, these things are only in 1.7.
  //vis_->setShowFPS(false);
  
  // -- Find all the example inducted datasets for the given class.
  induction_paths_.clear();
  bfs::recursive_directory_iterator it(root_), eod;
  string filename = "example-" + class_name_ + ".td";
  BOOST_FOREACH(const bfs::path& p, make_pair(it, eod)) {
    if(is_regular_file(p) && p.filename().string().compare(filename) == 0) {
      string path = p.string();
      string iternum = p.parent_path().filename().string().substr(4);  // starts with "iter".
      int iter = atoi(iternum.c_str());
      //cout << "Adding " << iter << " : " << path << endl;
      induction_paths_[iter] = path;
      max_iter_ = max(max_iter_, iter);
    }
  }
  ROS_ASSERT(!induction_paths_.empty());

  // -- Find all the user annotations.
  annotation_paths_.clear();
  bfs::recursive_directory_iterator it2(root_);
  string pos_filename = "annotated-" + class_name_ + "-pos.td";
  string neg_filename = "annotated-" + class_name_ + "-neg.td";
  BOOST_FOREACH(const bfs::path& p, make_pair(it2, eod)) {
    if(is_regular_file(p) &&
       (p.filename().string().compare(pos_filename) == 0 ||
        p.filename().string().compare(neg_filename) == 0))
    {
      string path = p.string();
      string iternum = p.parent_path().filename().string().substr(4);  // starts with "iter".
      int iter = atoi(iternum.c_str());
      annotation_paths_[iter].push_back(path);
    }
  }
  ROS_ASSERT(!annotation_paths_.empty());
}

std::vector<size_t> GroupInductionVisualizer::proportionalSample(const TrackDataset& td,
                                                                 size_t num_slots,
                                                                 size_t representation,
                                                                 int td_size_override) const
{
  ROS_ASSERT(num_slots > 0);
  ROS_ASSERT(representation >= 1);
  size_t td_size = td.size();
  if(td_size_override >= 0)
    td_size = td_size_override;
    
  size_t num = min<size_t>(num_slots, ceil(0.5 + (double)td_size / representation));
  if(num > td.size())
    num = td.size();

  vector<size_t> indices(td.size());
  for(size_t i = 0; i < td.size(); ++i)
    indices[i] = i;
  random_shuffle(indices.begin(), indices.end());
  indices.resize(num);

  return indices;
}

void GroupInductionVisualizer::loadAnnotations(const std::vector<std::string>& paths)
{
  for(size_t i = 0; i < paths.size(); ++i) {
    TrackDataset td;
    td.load(paths[i]);
    if(annotation_td_.empty())
      annotation_td_.applyNameMappings(td);
    annotation_td_ += td;
  }
  cout << "Num annotations: " << annotation_td_.size() << endl;
}

bool GroupInductionVisualizer::incrementIter()
{
  do {
    ++iter_;
    ROS_ASSERT(iter_ >= 0);
    if(annotation_paths_.count(iter_) && annotation_paths_.find(iter_)->second.size() > 0)
      loadAnnotations(annotation_paths_[iter_]);
    
  } while(!induction_paths_.count(iter_) && iter_ < max_iter_);
  
  if(iter_ >= max_iter_)
    return false;
  else
    return true;
}

void GroupInductionVisualizer::initializeIter()
{
  ROS_ASSERT(induction_paths_.count(iter_));
  cout << "Loading " << induction_paths_[iter_] << "." << endl;
  induction_td_.load(induction_paths_[iter_]);
  cout << "Done." << endl;
  int num_pos, num_neg;
  getNumInductedTracks(&num_pos, &num_neg);
  induction_indices_ = proportionalSample(induction_td_,
                                          pow(induction_grid_width_, 2),
                                          induction_representation_,
                                          num_pos + num_neg);
  annotation_indices_ = proportionalSample(annotation_td_,
                                           pow(induction_grid_width_, 2),
                                           induction_representation_);
  frame_id_ = 0;
}

Cloud::Ptr GroupInductionVisualizer::generateGridLines() const
{
  Cloud::Ptr grid(new Cloud);
  float interval = meters_per_cell_ / 5;
  float min = 0;
  float max = induction_grid_width_ * meters_per_cell_;
  Point pt;
  pt.r = 127;
  pt.g = 127;
  pt.b = 127;
  pt.z = 0;
  
  for(size_t r = 0; r <= induction_grid_width_; ++r) {
    pt.y = r * meters_per_cell_;
    for(pt.x = min; pt.x <= max; pt.x += interval)
      grid->push_back(pt);
  }
  for(size_t c = 0; c <= induction_grid_width_; ++c) {
    pt.x = c * meters_per_cell_;
    for(pt.y = min; pt.y <= max; pt.y += interval)
      grid->push_back(pt);
  }
  return grid;
}

void colorize(bool pos, int val, Point* pt)
{
  double u = (double)val / 255.;
  pt->b = val;
  if(pos) {
    pt->g = (0.25 + 0.75 * u) * 255;
    pt->r = val;
  }
  else {
    pt->g = val;
    pt->r = (0.5 + 0.5 * u) * 255;
  }
}

void GroupInductionVisualizer::trackIndexToGridElement(int idx, size_t* row, size_t* col) const
{
  ROS_ASSERT(induction_grid_width_ % 2 == 1);
  
  // -- Basic version.
  // *row = idx / induction_grid_width_;
  // *col = idx - *row * induction_grid_width_;

  // -- Add tracks starting in the middle, then spreading outward.
  int orig_idx = idx;
  int center = (int)induction_grid_width_ / 2 + 1;
  int r = 0, c = 0;
  int ring;
  bool done = false;
  for(ring = 0; ring <= (int)induction_grid_width_ / 2; ++ring) { 
    for(r = center - ring; r <= center + ring; ++r) { 
      for(c = center - ring; c <= center + ring; ++c) {
        if((r == center - ring) || (r == center + ring) ||
           (c == center - ring) || (c == center + ring))
        {
          if(idx == 0) {
            done = true;
            break;
          }
          --idx;
        }
      }
      if(done) break;
    }
    if(done) break;
  }

  if(idx != 0) {
    cout << "orig_idx: " << orig_idx << endl;
    cout << "idx: " << idx << ".  Should be zero." << endl;
    cout << "r: " << r << endl;
    cout << "c: " << c << endl;
    cout << "ring: " << ring << endl;
  }
  ROS_ASSERT(idx == 0);
  *row = r;
  *col = c;
}

Cloud::Ptr GroupInductionVisualizer::generateGridContent(TrackDataset& td,
                                                         const std::vector<size_t>& indices,
                                                         size_t frame_id) const
{
  Cloud::Ptr grid(new Cloud);
  if(td.empty())
    return grid;
  
  int cidx = td.nameMapping("cmap").toId(class_name_);
  for(size_t i = 0; i < indices.size(); ++i) {
    size_t idx = indices[i];
    size_t row, col;
    trackIndexToGridElement(i, &row, &col);

    Dataset& track = td[idx];
    Cloud::Ptr pcd = boost::any_cast<Cloud::Ptr>(track[frame_id % track.size()].raw());
    Vector4f centroid;
    pcl::compute3DCentroid(*pcd, centroid);
    Vector4f offset = Vector4f::Zero();
    offset.x() = meters_per_cell_ * (col - 1) + meters_per_cell_ / 2;
    offset.y() = meters_per_cell_ * (induction_grid_width_ - row) + meters_per_cell_ / 2;
    Point min_pt, max_pt;
    pcl::getMinMax3D(*pcd, min_pt, max_pt);
    offset.z() = centroid.z() - min_pt.z;
    centroid -= offset;
    Cloud demeaned;
    pcl::demeanPointCloud(*pcd, centroid, demeaned);

    // -- Colorize.
    for(size_t j = 0; j < demeaned.size(); ++j) {
      Point& pt = demeaned[j];
      colorize((track.label()(cidx) > 0), pt.r, &pt);
    }
    
    *grid += demeaned;
  }

  return grid;
}

double ceilToOddSquared(double val)
{
  double odd = 1;
  double result = 0;
  while(val > result + 1e-6) {
    result = odd * odd;
    odd += 2;
  }
  return result;
}

void GroupInductionVisualizer::run()
{
  if(write_video_) { 
    if(!bfs::exists(output_dir_))
      bfs::create_directory(output_dir_);
    if(!bfs::exists(output_dir_ + "/images"))
      bfs::create_directory(output_dir_ + "/images");
  }
  
  // -- Generate elements that will not change.
  Cloud::Ptr grid = generateGridLines();
  vis_->updatePointCloud(grid, "induction-grid");
  vis_->updatePointCloud(grid, "annotation-grid");
  vis_->addText("Inducted data sample", 1777/6, 950, 32, 0.9, 0.9, 0.9,
                "induction-title", induction_viewport_id_);
  vis_->addText("User-annotated data sample", 1777/7, 950, 32, 0.9, 0.9, 0.9,
                "annotation-title", annotation_viewport_id_);

  // -- Set up for main loop.
  int frame_num = 0;
  int prev_num_inducted_pos = 0;
  int prev_num_inducted_neg = 0;
  int prev_num_annotated_pos = 0;
  int prev_num_annotated_neg = 0;
  while(iter_ < max_iter_) {
    if(!incrementIter())
      break;
    initializeIter();

    // -- Update counts for this iteration.
    // Inductions
    int num_inducted_pos, num_inducted_neg;
    getNumInductedTracks(&num_inducted_pos, &num_inducted_neg);
    int num_inducted_pos_added = max(0, num_inducted_pos - prev_num_inducted_pos);
    int num_inducted_neg_added = max(0, num_inducted_neg - prev_num_inducted_neg);
    prev_num_inducted_pos = num_inducted_pos;
    prev_num_inducted_neg = num_inducted_neg;

    // Annotations
    int num_annotated_pos = 0;
    int num_annotated_neg = 0;
    int cidx = induction_td_.nameMapping("cmap").toId(class_name_);
    for(size_t i = 0; i < annotation_td_.size(); ++i) {
      if(annotation_td_[i].label()(cidx) > 0)
        ++num_annotated_pos;
      else if(annotation_td_[i].label()(cidx) < 0)
        ++num_annotated_neg;
    }
    int num_annotated_pos_added = max(0, num_annotated_pos - prev_num_annotated_pos);
    int num_annotated_neg_added = max(0, num_annotated_neg - prev_num_annotated_neg);
    prev_num_annotated_pos = num_annotated_pos;
    prev_num_annotated_neg = num_annotated_neg;

    // -- Write out text that is constant for this iteration.
    {
      ostringstream oss;
      oss << "Number of inducted tracks (positive): " << num_inducted_pos << endl;
      oss << "Number of inducted tracks (negative): " << num_inducted_neg << endl;
      int num_pos, num_neg;
      getNumAutoAnnotated(&num_pos, &num_neg);
      oss << "Number of automatically-labeled negative examples: " << num_neg << endl;
      oss << "Iteration: " << iter_ << endl;
      vis_->removeShape("induction-text", induction_viewport_id_);
      vis_->addText(oss.str(), 10, 50, 22, 0.9, 0.9, 0.9, "induction-text", induction_viewport_id_);
    }

    {
      ostringstream oss;
      oss << "Number of annotated tracks: " << annotation_td_.size() << endl;
      oss << "Iteration: " << iter_ << endl;
      vis_->removeShape("annotation-text", annotation_viewport_id_);
      vis_->addText(oss.str(), 10, 50, 22, 0.9, 0.9, 0.9, "annotation-text", annotation_viewport_id_);
    }

    // -- Main loop for this iteration.
    for(size_t i = 0; i < num_frames_per_iter_; ++i) {
      
      // -- Update induction viewport.
      Vector4f induction_centroid;
      {
        Cloud::Ptr induction_pcd = generateGridContent(induction_td_,
                                                       induction_indices_,
                                                       frame_id_);
        vis_->updatePointCloud(induction_pcd, "induction");

        vis_->removeShape("increment-induction-pos", induction_viewport_id_);
        vis_->removeShape("increment-induction-neg", induction_viewport_id_);
        if(num_inducted_pos_added != 0) {
          ostringstream oss;
          oss << "+" << num_inducted_pos_added << endl;
          double val = 1.0 - min(1.0, (double)i / (num_frames_per_iter_ / 1.0));
          vis_->addText(oss.str(), 10, 150, 32, 0.0, val, 0.0,
                        "increment-induction-pos", induction_viewport_id_);
        }
        if(num_inducted_neg_added != 0) {
          ostringstream oss;
          oss << "+" << num_inducted_neg_added << endl;
          double val = 1.0 - min(1.0, (double)i / (num_frames_per_iter_ / 1.0));
          vis_->addText(oss.str(), 150, 150, 32, val, 0.0, 0.0,
                        "increment-induction-neg", induction_viewport_id_);
        }

        pcl::compute3DCentroid(*induction_pcd, induction_centroid);
      }
      
      // -- Update annotation viewport.
      {
        Cloud::Ptr annotation_pcd = generateGridContent(annotation_td_,
                                                        annotation_indices_,
                                                        frame_id_);
        vis_->updatePointCloud(annotation_pcd, "annotation");

        vis_->removeShape("increment-annotation-pos", annotation_viewport_id_);
        vis_->removeShape("increment-annotation-neg", annotation_viewport_id_);
        if(num_annotated_pos_added != 0) {
          ostringstream oss_pos;
          oss_pos << "+" << num_annotated_pos_added << endl;
          double val = 1.0 - min(1.0, (double)i / (num_frames_per_iter_ / 1.0));
          vis_->addText(oss_pos.str(), 10, 100, 32, 0, val, 0,
                        "increment-annotation-pos", annotation_viewport_id_);
        }
        if(num_annotated_neg_added != 0) {
          ostringstream oss_neg;
          oss_neg << "+" << num_annotated_neg_added << endl;
          double val = 1.0 - min(1.0, (double)i / (num_frames_per_iter_ / 1.0));
          vis_->addText(oss_neg.str(), 150, 100, 32, val, 0, 0,
                        "increment-annotation-neg", annotation_viewport_id_);
        }
      }

      // -- Global updates.
      if(radius_method_ == "overview") {
        double fov = 30.0 * M_PI / 180.0;
        double w = sqrt(ceilToOddSquared(max<double>(induction_indices_.size(), 1)));
        radius_setpoint_ = w * sqrt(2.0) / (2.0 * tan(fov / 2.0)) * meters_per_cell_;
      }
      else if(radius_method_ == "detail") {
        radius_setpoint_ = (2 + 0.5 * sqrt(ceilToOddSquared(max<double>(induction_indices_.size(), 1)))) * meters_per_cell_;
      }
      else
        ROS_ASSERT(0);

      radius_ = 0.95 * radius_ + 0.05 * radius_setpoint_;
      setCameraPose();
      vis_->spinOnce(ms_per_frame_);

      if(write_video_) {
        ostringstream oss;
        oss << output_dir_ << "/images/gi-" << setw(5) << setfill('0') << frame_num << ".png";
        vis_->saveScreenshot(oss.str());
        ++frame_num;
      }
      
      theta_ += rotation_speed_;
      pos_z_multiplier_ = 0.5;
      ++frame_id_;
    }
  }

  if(write_video_) {
    /* A good generic video assembly command is this:
     * mencoder mf://gi-*.png -mf fps=30 -ovc x264 -x264encopts crf=13 -vf scale=1280:720 -o test.avi
     * Here I'm basically using this, but changing the resolution to match the screen size set
     * for PCLVis.
     */
    ostringstream oss;
    oss << "mencoder \"mf://" << output_dir_
        << "/images/*.png\" -mf fps=" << frames_per_second_
        << " -ovc x264 -x264encopts crf=13 -vf scale=1777:1000"
        << " -o " << output_dir_ << "/video.avi";
    
    cout << "Running command: " << endl;
    cout << "  " << oss.str() << endl;
    int retval = system(oss.str().c_str());
    if(retval != 0) {
      ROS_WARN_STREAM("Video generation command failed.  retval: " << retval);
      ROS_WARN_STREAM("  Line: " << oss.str());
    }
  }
}

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  string run_dir;
  string class_name;
  double meters_per_cell;
  size_t induction_grid_width;
  string radius_method;
  int max_iter;
  string output_dir;
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("run-dir,r", bpo::value(&run_dir)->required(), "group induction run directory")
    ("class,c", bpo::value(&class_name)->required(), "which class to show")
    ("meters-per-cell", bpo::value(&meters_per_cell)->required())
    ("induction-grid-width", bpo::value(&induction_grid_width)->default_value(5))
    ("radius-method", bpo::value(&radius_method)->default_value("overview"), "\"overview\" or \"detail\"")
    ("max-iter", bpo::value(&max_iter)->default_value(1e6))
    ("output-dir", bpo::value(&output_dir)->default_value("group_induction_video"))
    ("write-video", "")
    ;

  p.add("run-dir", 1);
  p.add("class", 1);
  p.add("meters-per-cell", 1);

  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << argv[0] << " [OPTS] RUN_DIR CLASS" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  ROS_ASSERT(induction_grid_width % 2 == 1);
  
  GroupInductionVisualizer giv(run_dir, class_name, meters_per_cell);
  giv.induction_grid_width_ = induction_grid_width;
  giv.radius_method_ = radius_method;
  giv.max_iter_ = max_iter;
  giv.output_dir_ = output_dir;
  giv.write_video_ = opts.count("write-video");

  giv.run();
  return 0;
}
