#include <fstream>
#include <yaml-cpp/yaml.h>

using namespace std;

int main(int argc, char** argv)
{
  YAML::Node doc = YAML::LoadFile("data/example.yml");

  const YAML::Node& pods = doc["Pods"];
  for(size_t i = 0; i < pods.size(); ++i) {
    const YAML::Node& pod = pods[i];

    cout << " --- " << endl;
    cout << pod["Name"].as<string>() << endl;
    cout << pod["Type"].as<string>() << endl;

    if(pod["Inputs"]) {
      const YAML::Node& inputs = pod["Inputs"];
      for(YAML::const_iterator it = inputs.begin(); it != inputs.end(); ++it) { 
        cout << it->first << " <- " << it->second << endl;
      }
    }
  }
  
  return 0;
}
  
