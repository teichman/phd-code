#include <eigen_extensions/eigen_extensions.h>
#include <xpl_calibration/calibration_pipeline_orb.h>
#include <xpl_calibration/calibration_pipeline_dynamic.h>

using namespace std;
using namespace pcl;
using namespace rgbd;

#define NUM_THREADS (getenv("NUM_THREADS") ? atoi(getenv("NUM_THREADS")) : 1)

string usageString()
{
  ostringstream oss;
  oss << "Usage: calibrate_sequences MODE SEQ0 SEQ1 BASENAME [PIPELINE]" << endl;
  oss << "  where MODE is --orb or --dynamic." << endl;
  oss << "  A transform & sync will be output to transform.eig.txt and sync.eig.txt which should transform SEQ1 to SEQ0." << endl;
  return oss.str();
}

int main(int argc, char** argv)
{
  if(argc != 4 && argc != 5) {
    cout << usageString() << endl;
    return 0;
  }

  string mode = argv[1];
  string path0 = argv[2];
  string path1 = argv[3];
  if(mode.compare("--orb") == 0) 
    cout << "Using orb method." << endl;
  else if(mode.compare("--dynamic") == 0)
    cout << "Using dynamic method." << endl;
  else {
    cout << "Unknown mode " << mode << endl;
    cout << endl;
    cout << usageString() << endl;
    return 0;
  }
  
  StreamSequence::Ptr sseq0(new StreamSequence);
  StreamSequence::Ptr sseq1(new StreamSequence);
  cout << "Loading " << path0 << "." << endl;
  sseq0->load(path0);
  cout << "Loading " << path1 << "." << endl;
  sseq1->load(path1);

  string pipeline_path = "";
  if(argc == 5) { 
    pipeline_path = argv[4];
    cout << "Using custom Pipeline specification: " << pipeline_path << endl;
  }
  
  Eigen::Affine3f transform;
  if(mode.compare("--orb") == 0) { 
    CalibrationPipelineOrb cp(NUM_THREADS, pipeline_path);
    transform = cp.calibrate(sseq0, sseq1);
  }
  else { 
    CalibrationPipelineDynamic cp(NUM_THREADS, pipeline_path);
    double sync;
    cp.calibrate(sseq0, sseq1, &transform, &sync);
    cout << "Sync offset: " << sync << endl;
    Eigen::VectorXd vdt(1);
    vdt(0) = sync;
    eigen_extensions::saveASCII(vdt, "sync.eig.txt");
  }
  cout << transform.matrix() << endl;
  eigen_extensions::saveASCII(transform.matrix(), "transform.eig.txt");
  
  return 0;
}
