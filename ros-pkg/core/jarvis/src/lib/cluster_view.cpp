#include <algorithm>

#include <agent/lockable.h>
#include <boost/any.hpp>
#include <jarvis/cluster_view_controller.h>
#include <jarvis/texture.h>
#include <jarvis/tracker.h>
#include <online_learning/dataset.h>

#include <GL/gl.h>
#include <GL/glut.h>

using namespace std;

static const float kSidePadding = 0.1f;
static const float kBetweenPadding = 0.1f;
static const float kClusterDistance = 1.0f + kBetweenPadding;
static const float kListWidth = 1.0 + 2 * kSidePadding;
static const float kListHeight = 16.0/9.0 * 320/240 * kListWidth;

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


void ClusterListView::display() {
  glViewport(vp_.x, vp_.y, vp_.w, vp_.h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  // each cluster is displayed in unit rectangle (0,0)-(1,1)
  glOrtho(0, kListWidth, 0, kListHeight, -1, 1);
  glDisable(GL_DEPTH_TEST);
  glDisable(GL_BLEND);

  scopeLockRead;
  float scroll_fraction = scroll_position_ - floorf(scroll_position_);
  size_t max_displayed = ceilf(
      (kListHeight + scroll_fraction) / kClusterDistance);
  max_displayed = min(clusters_.size() - (int) ceilf(scroll_position_),
                      max_displayed);
  for (size_t i = 0; i < max_displayed; i++)
  {
    glPushMatrix();
    glTranslatef(
        kSidePadding,
        kListHeight - 1
            + (scroll_fraction - i)
                * kClusterDistance,
        0);
    clusters_[i+(int)(scroll_position_)]->display();
    glPopMatrix();
  }
  glutPostRedisplay();
}

void ClusterListView::motion(int x, int y) {
  if (!dragging_)
    return;
  int motion_y = y - last_mouse_y_;
  last_mouse_y_ = y;

  scopeLockWrite;
  float scroll_motion = motion_y / (float)vp_.h * kListHeight / kClusterDistance;
//  cerr << "motion: " << motion_y << " to scroll " << scroll_motion;
  scroll_position_ += scroll_motion;
}

void ClusterListView::mouse(int button, int button_state, int x, int y) {
  if (button == GLUT_LEFT_BUTTON)
  {
    dragging_ = (button_state == GLUT_DOWN);
    if (dragging_)
      last_mouse_y_ = y;
    cout << "dragging: " << dragging_ << endl;
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
  clusters_.push_back(ClusterView::Ptr(new ClusterView(cluster)));
}

int ClusterListView::indexPicked(int mouse_x, int mouse_y) {
  scopeLockRead;
  return -1;
}

