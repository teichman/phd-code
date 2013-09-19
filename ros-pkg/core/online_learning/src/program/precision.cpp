#include <iostream>

using namespace std;

int main(int argc, char** argv)
{
  float fweight = 1.45;
  float flearning_rate = 1e-12;
  double update = fweight * flearning_rate;
  cout << "update using floats: " << update << endl;
  
  return 0;
}
