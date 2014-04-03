#include <iostream>
#include <unistd.h>

using namespace std;

int main(int argc, char** argv)
{
  cout << sysconf(_SC_NPROCESSORS_ONLN) << endl;
  return 0;
}
