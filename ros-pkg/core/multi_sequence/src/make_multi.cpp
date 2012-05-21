#include <multi_sequence/multi_sequence.h>

using namespace std;
using namespace multi_sequence;

string usageString()
{
  ostringstream oss;
  oss << "Usage: make_multi output dt_thresh dir1 [dir2] [dir3] ..." << endl;
  oss << "This copies all data" << endl;
  return oss.str();
}

int main(int argc, char** argv)
{
  if(argc < 4) {
    cout << usageString() << endl;
    return 0;
  }
  
  string output_dir = argv[1];
  float dt = atof(argv[2]);
  vector<rgbd::StreamSequence::Ptr> seqs(argc-3);
  for(int i = 0; i < argc-3; i++ ){
    seqs[i] = rgbd::StreamSequence::Ptr( new rgbd::StreamSequence );
    seqs[i]->load(argv[i+3]);
  }
  MultiSequence mseq(dt, seqs);
  mseq.save(output_dir);
  return 0;
}



