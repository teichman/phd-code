#include <rgbd_sequence/openni_stream_recorder.h>

using namespace rgbd;

int main(int argc, char** argv)
{
  OpenNIStreamRecorder rec;
  rec.run();

  return 0;
}
