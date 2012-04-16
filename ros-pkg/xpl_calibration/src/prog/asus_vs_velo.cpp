#include <xpl_calibration/asus_vs_velo_visualizer.h>


using namespace std;
using namespace pcl;
using namespace rgbd;

string usageString()
{
  ostringstream oss;
  oss << "Usage: asus_vs_velo SSEQ VSEQ [CAL]" << endl;
  return oss.str();
}
  

int main(int argc, char** argv)
{
  if(argc != 3 && argc != 4) {
    cout << usageString() << endl;
    return -1;
  }
  
  StreamSequence::Ptr sseq(new StreamSequence);
  sseq->load(argv[1]);

  VeloSequence::Ptr vseq(new VeloSequence(argv[2]));
  AsusVsVeloVisualizer avv(sseq, vseq);
  if(argc == 4) {
    avv.cal_.load(argv[3]);
    cout << "Loaded calibration " << argv[3] << "." << endl;
    cout << avv.cal_.offset_ << endl;
    cout << avv.cal_.velo_to_asus_.matrix() << endl;
  }
  avv.run();
  
  return 0;
}
