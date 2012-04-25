#include <rgbd_sequence/stream_sequence.h>

using namespace std;
using namespace rgbd;

string usageString()
{
  ostringstream oss;
  oss << "Usage: offset_clock SEQ_IN SEQ_OUT OFF" << endl;
  oss << "Where SEQ_IN is the input sequence" << endl;
  oss << "Where SEQ_OUT is the output sequence" << endl;
  oss << "Where OFF is the offset in seconds" << endl;
  return oss.str();
}

int main(int argc, char** argv)
{
  if(argc < 4) {
    cout << usageString() << endl;
    return 0;
  }
  string seq_in = argv[1];
  string seq_out = argv[2];
  float offset = atof(argv[3]);
  cout << "Looking at seq: " << seq_in << endl;
  
  StreamSequence seq;
  seq.load(seq_in);
  cout << "Loaded successfully" << endl;
  for(size_t i = 0; i < seq.timestamps_.size(); i++){
    seq.timestamps_[i] += offset;
  }
  seq.save(seq_out);
  cout << "Saved to " << seq_out << endl;
  return 0;
}

