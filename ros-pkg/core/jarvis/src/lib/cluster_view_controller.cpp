#include <algorithm>

#include <agent/lockable.h>
#include <boost/any.hpp>
#include <jarvis/cluster_view_controller.h>
#include <online_learning/clusterer.h>
#include <jarvis/texture.h>
#include <jarvis/tracker.h>
#include <online_learning/tbssl.h>
#include <online_learning/dataset.h>

#include <GL/gl.h>
#include <GL/glut.h>

using namespace std;

/************************************************************
 * ClusterBlobView
 ************************************************************/

ClusterBlobView::ClusterBlobView() :
  instance_displayed_(0)
{
}

void ClusterBlobView::displayCluster(TrackDataset::Ptr td)
{
  cout << "  Displaying cluster with " << td->size() << " tracks." << endl;
  scopeLockWrite;
  td_ = td;
}

void ClusterBlobView::displayMessage(const std::string& message)
{
  cout << "  Displaying message: " << message << endl;
}

void ClusterBlobView::display() {
  if (td_.get() == NULL)
    return;
  glViewport(vp_.x, vp_.y, vp_.w, vp_.h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(0, 1, 0, 1, -1, 1);

  Texture tex;
  vector<uint8_t> img;

  scopeLockWrite;
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE);
  for(Dataset::Ptr t : td_->tracks_) {
    ROS_ASSERT(t);
    ROS_ASSERT(t->instances_.size() > 0);
    size_t frame = min(instance_displayed_, t->instances_.size()-1);
    Blob::ConstPtr blob = boost::any_cast<Blob::ConstPtr>(
      t->instances_[frame].raw());
    ROS_ASSERT(blob.get() != NULL);
    img.resize(blob->width_ * blob->height_ * 4);
    tex.Resize(GL_RGBA, blob->width_, blob->height_);
    memset(img.data(), 0, img.size());
    uint8_t alpha = min(255, (int)(500.0 / pow(td_->tracks_.size(), 0.7)));
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
  instance_displayed_ = (instance_displayed_ + 1) % 150;
  glutPostRedisplay();
}

void ClusterBlobView::key(unsigned char k, int x, int y) {
  cout << "Got key: " << k << endl;
//  cvc_->handleKeypress(k);
}


