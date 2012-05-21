#include <xpl_calibration/asus_vs_velo_visualizer.h>


using namespace std;
using namespace pcl;
using namespace rgbd;

string usageString()
{
  ostringstream oss;
  oss << "Usage: asus_vs_velo SSEQ VSEQ [CAL] [DEPTH_DISTORTION_MODEL]" << endl;
  return oss.str();
}
  

int main(int argc, char** argv)
{
  if(argc != 3 && argc != 4 && argc != 5) {
    cout << usageString() << endl;
    return -1;
  }
  
  StreamSequence::Ptr sseq(new StreamSequence);
  sseq->load(argv[1]);

  VeloSequence::Ptr vseq(new VeloSequence(argv[2]));
  AsusVsVeloVisualizer avv(sseq, vseq);
  if(argc == 4 || argc == 5) {
    avv.cal_.load(argv[3]);
    cout << "Loaded calibration " << argv[3] << "." << endl;
    cout << avv.cal_.offset_ << endl;
    cout << avv.cal_.velo_to_asus_.matrix() << endl;
  }
  if(argc == 5) {
    eigen_extensions::loadASCII(argv[4], &avv.weights_);
    cout << "Loaded depth distortion model at " << argv[4] << endl;
    cout << "Weights: " << avv.weights_.transpose() << endl;
  }
  avv.run();
  
  return 0;
}
