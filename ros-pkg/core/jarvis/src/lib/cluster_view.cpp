#include <algorithm>

#include <agent/lockable.h>
#include <boost/any.hpp>
#include <jarvis/cluster_view_controller.h>
#include <jarvis/texture.h>
#include <jarvis/tracker.h>
#include <online_learning/dataset.h>
#include <timer/timer.h>

#include <GL/gl.h>

using namespace std;

static const float kSidePadding = 0.1f;
static const float kBetweenPadding = 0.1f;
static const float kClusterDistance = 1.0f + kBetweenPadding;
static const float kListWidth = 1.0 + 2 * kSidePadding;
static const float kListHeight = 16.0/9.0 * 320/240 * kListWidth;
static const float kMomentumDecel = 6.0f;  // clusters/sec/sec

ClusterView::ClusterView(boost::shared_ptr<TrackDataset> cluster)
    : cluster_(cluster), instance_displayed_(0), max_instances_(0)
{
  // Check for empty tracks.  This shouldn't be happening but appears to be anyway.
  int num_empty = 0;
  for (Dataset::Ptr track : cluster_->tracks_)
  {
    ROS_ASSERT(track);
    if (track->empty())
      ++num_empty;
    max_instances_ = max(max_instances_, track->size());
  }
  if (num_empty > 0)
    ROS_WARN_STREAM(
        "ClusterView received a cluster that contains " << num_empty << " empty tracks.");
}

void ClusterView::display()
{
  if (cluster_.get() == NULL)
    return;

  Texture tex;
  vector<uint8_t> img;

  glColor4f(0, 0, 0, 0.5f);
  glBegin (GL_QUADS);
  glVertex3f(0, 0, 0);
  glVertex3f(0, 1, 0);
  glVertex3f(1, 1, 0);
  glVertex3f(1, 0, 0);
  glEnd();

  glColor3f(1, 1, 1);

  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE);
  for(Dataset::Ptr t : cluster_->tracks_) {
    ROS_ASSERT(t);
    if(t->instances_.empty())
      continue;

    size_t frame = min(instance_displayed_, t->instances_.size()-1);
    Blob::ConstPtr blob = boost::any_cast<Blob::ConstPtr>(
      t->instances_[frame].raw());
    ROS_ASSERT(blob.get() != NULL);
    img.resize(blob->width_ * blob->height_ * 4);
    tex.Resize(GL_RGBA, blob->width_, blob->height_);
    memset(img.data(), 0, img.size());
    uint8_t alpha = min(255, (int)(500.0 / pow(cluster_->tracks_.size(), 0.7)));
    for (size_t i = 0; i < blob->indices_.size(); i++) {
      uint32_t idx = 4 * blob->indices_[i];
      img[idx + 0] = blob->color_[3 * i + 0];
      img[idx + 1] = blob->color_[3 * i + 1];
      img[idx + 2] = blob->color_[3 * i + 2];
      img[idx + 3] = alpha;
    }
    tex.CopyToGPU(GL_RGBA, img.data());
    tex.DrawAsQuad();
  }
  glDisable(GL_BLEND);
  instance_displayed_ = (instance_displayed_ + 1) % max_instances_;
}

ClusterListView::ClusterListView()
    : scroll_position_(0), scroll_momentum_(0),
        scroll_last_update_(new HighResTimer()), dragging_(false),
        last_mouse_y_(0)
{
  scroll_last_update_->start();
}


void ClusterListView::display() {
  glViewport(vp_.x, vp_.y, vp_.w, vp_.h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  // each cluster is displayed in unit rectangle (0,0)-(1,1)
  glOrtho(0, kListWidth, 0, kListHeight, -1, 1);
  glDisable(GL_DEPTH_TEST);
  glDisable(GL_BLEND);

  scopeLockRead;
  applyScrollMomentum();
  float scroll_fraction = scroll_position_ - floorf(scroll_position_);
  size_t max_displayed = ceilf(
      (kListHeight + scroll_fraction) / kClusterDistance);
  max_displayed = min(clusters_.size() - (int) ceilf(scroll_position_),
                      max_displayed);
  for (size_t i = 0; i < max_displayed; i++)
  {
    glPushMatrix();
    glTranslatef(kSidePadding,
                 kListHeight - 1
                 + (scroll_fraction - i) * kClusterDistance,
                 0);
    clusters_[i+(int)(scroll_position_)]->display();
    glPopMatrix();
  }
  PostRedisplay();
}

void ClusterListView::applyScrollMotion(float scroll_motion) {
  float max_scroll = max(0.0f,
      ((float) clusters_.size() - kListHeight - 1)) * kClusterDistance;

  scroll_position_ += scroll_motion;
  if (scroll_position_ > max_scroll) {
    scroll_position_ = max_scroll;
    scroll_momentum_ = 0;
  }
  if (scroll_position_ < 0) {
    scroll_position_ = 0;
    scroll_momentum_ = 0;
  }
  scroll_momentum_ = max(-10.0f, min(10.0f, scroll_momentum_));
}

void ClusterListView::applyScrollMomentum() {
  if (!dragging_ && fabs(scroll_momentum_) > 0.001) {
    float dt = scroll_last_update_->getSeconds();
    scroll_last_update_->reset();
    scroll_last_update_->start();
    float scroll_motion = scroll_momentum_ * dt;
    if (scroll_momentum_ > 0) {
      scroll_momentum_ = max(0.0f, scroll_momentum_ - kMomentumDecel * dt);
    } else {
      scroll_momentum_ = min(0.0f, scroll_momentum_ + kMomentumDecel * dt);
    }
//    fprintf(stderr, "      dt: %8.4f motion %8.4f momentum %8.4f\n", dt,
//            scroll_motion, scroll_momentum_);
    applyScrollMotion(scroll_motion);
  }
}

void ClusterListView::motion(int x, int y) {
  if (!dragging_)
    return;
  scopeLockRead;
  int motion_y = y - last_mouse_y_;
  last_mouse_y_ = y;
  float dt = scroll_last_update_->getSeconds();
  scroll_last_update_->reset();
  scroll_last_update_->start();

  float scroll_motion = motion_y / (float)vp_.h * kListHeight / kClusterDistance;
  scroll_momentum_ = max(0.0f, (1.0f - 3*dt)) * scroll_momentum_
                   + 3 * scroll_motion;
  applyScrollMotion(scroll_motion);
}

void ClusterListView::mouse(int button, int button_state, int x, int y) {
  scopeLockRead;
  switch (button) {
  case 0:  // left mouse button
    dragging_ = (button_state == 0);
    if (dragging_)
      last_mouse_y_ = y;
    break;
  case 3: // scroll up
  case 4: // scroll down
    if (button_state == 0) {
      scroll_last_update_->reset();
      scroll_last_update_->start();
      float tmp = fabs(scroll_momentum_) * scroll_momentum_
          + ((button == 4) ? 10 : -10);
      scroll_momentum_ = (tmp > 0) ? sqrt(tmp) : -sqrt(-tmp);
    }
    break;
  }
}

void ClusterListView::clear() {
  scopeLockWrite;
  clusters_.clear();
}

int ClusterListView::clustersBeforeEnd() {
  return clusters_.size() -
      (scroll_position_ + (kListHeight / kClusterDistance));
}

void ClusterListView::displayCluster(TrackDataset::Ptr cluster) {
  cout << "  Displaying cluster with " << cluster->size() << " tracks." << endl;
  scopeLockWrite;
  ROS_ASSERT(cluster->size() > 0);
  if (displayed_datasets_.find(cluster->tracks_[0]->hash())
      == displayed_datasets_.end()) {
    clusters_.push_back(ClusterView::Ptr(new ClusterView(cluster)));
    for (Dataset::Ptr track : cluster->tracks_) {
      displayed_datasets_.insert(track->hash());
    }
  } else {
    cerr << "warning: cluster already displayed, not adding";
  }
}

int ClusterListView::indexPicked(int mouse_x, int mouse_y)
{
  scopeLockRead;
  float picked = scroll_position_ + ((vp_.h - mouse_y) /
      (float)vp_.h * kListHeight + kBetweenPadding/2.0f) / kClusterDistance;
  return (int)picked;
}

