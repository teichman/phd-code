#include <jarvis/cluster_view_controller.h>

using namespace std;


/************************************************************
 * ClusterBlobView
 ************************************************************/

ClusterBlobView::ClusterBlobView(ClusterViewController* cvc) :
  cvc_(cvc)
{
}

void ClusterBlobView::displayCluster(TrackDataset td)
{
  cout << "Displaying cluster with " << td.size() << " tracks." << endl;
}

void ClusterBlobView::displayMessage(const std::string& message)
{
  cout << "Displaying message: " << message << endl;
}

void ClusterBlobView::_run()
{
  while(!quitting_) {
    usleep(1e6);
    cout << "[ClusterBlobView] spinning..." << endl;
  }
}



/************************************************************
 * ClusterViewController
 ************************************************************/


ClusterViewController::ClusterViewController(OnlineLearner* ol) :
  ol_(ol)
{
}

void ClusterViewController::handleKeypress(char key)
{

}

void ClusterViewController::_run()
{
  ROS_ASSERT(view_);
  
  view_->displayMessage("starting up");
  while(!quitting_) {
    usleep(1e6);
    //view_->displayCluster(td);
    cout << "[ClusterViewController] spinning..." << endl;
  }
}
  
