#include <rgbd_sequence/trc_parser.h>
#include <rgbd_sequence/vis_wrapper.h>

using namespace std;
using namespace rgbd;

int main(int argc, char** argv)
{
  TRCParser trc;
  trc.load(argv[1]);

  VisWrapper vw;
  for(size_t i = 0; i < trc.frames_.size(); ++i) {
    cout << "Showing frame " << i << " with " << trc.frames_[i]->size() << " points." << endl;
    
    vw.showCloud(trc.frames_[i]);
    vw.waitKey();
  }
  
  return 0;
}
