#include <vector>
#include <iostream>

using namespace std;

int main(int argc, char** argv)
{
  vector<int> foo(1e6, 15);
  vector<int>* data = new vector<int>(1e6, 13);
  cout << data << endl;
  vector<int> foo2(1e6, 15);
  
  return 0;
}
