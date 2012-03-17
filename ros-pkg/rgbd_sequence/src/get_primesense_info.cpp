#include <rgbd_sequence/stream_recorder.h>

using namespace std;
using namespace pcl;

string usageString()
{
  ostringstream oss;
  oss << "Usage: get_primesense_info" << endl;
  return oss.str();
}

int main(int argc, char** argv)
{
  OpenNIGrabber grabber("", pcl::OpenNIGrabber::OpenNI_VGA_30Hz, pcl::OpenNIGrabber::OpenNI_VGA_30Hz);
  openni_wrapper::OpenNIDevice* dev = grabber.getDevice().get();
  cout << "isDepthRegistered(): " << dev->isDepthRegistered() << endl;
  cout << "isDepthRegistrationSupported(): " << dev->isDepthRegistrationSupported() << endl;
  cout << "isSynchronized(): " << dev->isSynchronized() << endl;
  cout << "Image f: " << dev->getImageFocalLength(0) << endl;
  cout << "Depth f: " << dev->getDepthFocalLength(0) << endl;
  cout << "getVendorID(): " << dev->getVendorID() << endl;
  cout << "getProductID(): " << dev->getProductID() << endl;
  cout << "getVendorName(): " << dev->getVendorName() << endl;
  cout << "getProductName(): " << dev->getProductName() << endl;
  cout << "getSerialNumber(): " << dev->getSerialNumber() << endl;
  cout << "getBaseline(): " << dev->getBaseline() << endl;

  //grabber.start();
  dev->setDepthRegistration(true);
  dev->setSynchronization(true);

  cout << "----------------------------------------" << endl;
  cout << "isDepthRegistered(): " << dev->isDepthRegistered() << endl;
  cout << "isDepthRegistrationSupported(): " << dev->isDepthRegistrationSupported() << endl;
  cout << "isSynchronized(): " << dev->isSynchronized() << endl;
  cout << "Image f: " << dev->getImageFocalLength(0) << endl;
  cout << "Depth f: " << dev->getDepthFocalLength(0) << endl;
  
  return 0;
}
