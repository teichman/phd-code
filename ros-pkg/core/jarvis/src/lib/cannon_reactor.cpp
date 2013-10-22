#include <jarvis/cannon_reactor.h>
#include <online_learning/dataset.h>

using namespace std;

CannonReactor::CannonReactor()
{

}

void CannonReactor::detectionCallback(jarvis::DetectionConstPtr msg)
{
  NameMapping cmap(msg->cmap);
  Label pred(msg->label);
  cout << "Detection: " << endl;
  cout << "  " << pred.status(cmap) << endl;
}
