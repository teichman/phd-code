#include <xpl_calibration/asus_vs_velo_visualizer.h>


using namespace std;
using namespace pcl;
using namespace rgbd;

string usageString()
{
  ostringstream oss;
  oss << "Usage: asus_vs_velo SSEQ VSEQ" << endl;
  return oss.str();
}
  

int main(int argc, char** argv)
{
  if(argc != 3) {
    cout << usageString() << endl;
    return -1;
  }

  StreamSequence::Ptr sseq(new StreamSequence);
  sseq->load(argv[1]);

  VeloSequence::Ptr vseq(new VeloSequence(argv[2]));
  AsusVsVeloVisualizer avv(sseq, vseq);
  avv.run();
  
  return 0;
}
