#include <bag_of_tricks/high_res_timer.h>
#include <gtest/gtest.h>

using namespace std;

TEST(HighResTimer, HighResTimer)
{
  timespec t;
  t.tv_sec = 0;
  t.tv_nsec = 345098;
  
  HighResTimer hrt;
  hrt.start();
  nanosleep(&t, NULL);
  hrt.stop();
  cout << hrt.report() << endl;
}

TEST(ScopedTimer, ScopedTimer)
{
  HighResTimer hrt;
  timespec t;
  t.tv_sec = 0;
  t.tv_nsec = 1e6;

  {
    ScopedTimer st;
    hrt.start();
    nanosleep(&t, NULL);
  }
  hrt.stop();    
  cout << hrt.report() << endl;
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
