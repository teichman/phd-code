#ifndef ASPVIS_H
#define ASPVIS_H

#include <image_labeler/opencv_view.h>
#include <bag_of_tricks/agent.h>
#include <asp/asp.h>

namespace asp
{
  class AspVis : public Agent, public OpenCVViewDelegate
  {
  public:
    AspVis(Asp* asp, cv::Mat3b img, double scale = 1.0);

  protected:
    Asp* asp_;
    cv::Mat3b img_;
    OpenCVView img_view_;
    cv::Mat1b seed_;
    bool show_seed_;
    bool needs_redraw_;
    int seed_radius_;
    OpenCVView seg_view_;
    cv::Mat1b seg_;
    bool debug_;
    
    virtual void _run();
    void draw();
    void segment();
    void handleKeypress(char key);
    void mouseEvent(int event, int x, int y, int flags, void* param);
    
    friend class OpenCVView;
  };
}

#endif // ASPVIS_H
