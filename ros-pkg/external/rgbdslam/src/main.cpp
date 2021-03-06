/* This file is part of RGBDSLAM.
 * 
 * RGBDSLAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * RGBDSLAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with RGBDSLAM.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "openni_listener.h"
#include "qtros.h"
#include <QApplication>
#include <QObject>
#include "qt_gui.h"
#include <Eigen/Core>
#include "parameter_server.h"
#include "ros_service_ui.h"

//TODO:
//better potential-edge-selection through flann
//Better separation of function, communication, parameters and gui

///Connect Signals and Slots for the ui control
void ui_connections(QObject* ui, GraphManager* graph_mgr, OpenNIListener* listener)
{
    QObject::connect(ui, SIGNAL(reset()), graph_mgr, SLOT(reset()));
    QObject::connect(ui, SIGNAL(optimizeGraph()), graph_mgr, SLOT(optimizeGraph()));
    QObject::connect(ui, SIGNAL(togglePause()), listener, SLOT(togglePause()));
    QObject::connect(ui, SIGNAL(toggleBagRecording()), listener, SLOT(toggleBagRecording()));
    QObject::connect(ui, SIGNAL(getOneFrame()), listener, SLOT(getOneFrame()));
    QObject::connect(ui, SIGNAL(deleteLastFrame()), graph_mgr, SLOT(deleteLastFrame()));
    QObject::connect(ui, SIGNAL(sendAllClouds()), graph_mgr, SLOT(sendAllClouds()));
    QObject::connect(ui, SIGNAL(saveAllClouds(QString)), graph_mgr, SLOT(saveAllClouds(QString)));
    QObject::connect(ui, SIGNAL(saveIndividualClouds(QString)), graph_mgr, SLOT(saveIndividualClouds(QString)));
    QObject::connect(ui, SIGNAL(setMaxDepth(float)), graph_mgr, SLOT(setMaxDepth(float)));
    QObject::connect(ui, SIGNAL(saveTrajectory(QString)), graph_mgr, SLOT(saveTrajectory(QString)));
}

///Connect Signals and Slots only relevant for the graphical interface
void gui_connections(Graphical_UI* gui, GraphManager* graph_mgr, OpenNIListener* listener)
{
    QObject::connect(listener,  SIGNAL(newVisualImage(QImage)), gui, SLOT(setVisualImage(QImage)));
    QObject::connect(listener,  SIGNAL(newFeatureFlowImage(QImage)), gui, SLOT(setFeatureFlowImage(QImage)));
    QObject::connect(listener,  SIGNAL(newDepthImage(QImage)), gui, SLOT(setDepthImage(QImage)));
    QObject::connect(graph_mgr, SIGNAL(sendFinished()), gui, SLOT(sendFinished()));
    QObject::connect(graph_mgr, SIGNAL(setGUIInfo(QString)), gui, SLOT(setInfo(QString)));
    QObject::connect(graph_mgr, SIGNAL(setGUIStatus(QString)), gui, SLOT(setStatus(QString)));
    QObject::connect(gui, SIGNAL(printEdgeErrors(QString)), graph_mgr, SLOT(printEdgeErrors(QString)));
    QObject::connect(gui, SIGNAL(pruneEdgesWithErrorAbove(float)), graph_mgr, SLOT(pruneEdgesWithErrorAbove(float)));
    if (ParameterServer::instance()->get<bool>("use_glwidget") && gui->getGLViewer() != NULL) {
      GLViewer* glv = gui->getGLViewer();
            QObject::connect(graph_mgr, SIGNAL(setPointCloud(pointcloud_type *, QMatrix4x4)), glv, SLOT(addPointCloud(pointcloud_type *, QMatrix4x4))); //, Qt::DirectConnection);
            QObject::connect(graph_mgr, SIGNAL(setGraphEdges(QList<QPair<int, int> >*)), glv, SLOT(setEdges(QList<QPair<int, int> >*)));
            QObject::connect(graph_mgr, SIGNAL(updateTransforms(QList<QMatrix4x4>*)), glv, SLOT(updateTransforms(QList<QMatrix4x4>*)));
      QObject::connect(graph_mgr, SIGNAL(deleteLastNode()), glv, SLOT(deleteLastNode()));
            QObject::connect(graph_mgr, SIGNAL(resetGLViewer()),  glv, SLOT(reset()));
      if(!ParameterServer::instance()->get<bool>("store_pointclouds")) {
          QObject::connect(glv, SIGNAL(cloudRendered(pointcloud_type const *)), graph_mgr, SLOT(cloudRendered(pointcloud_type const *))); // 
      }
    }
    QObject::connect(listener, SIGNAL(setGUIInfo(QString)), gui, SLOT(setInfo(QString)));
    QObject::connect(listener, SIGNAL(setGUIStatus(QString)), gui, SLOT(setStatus(QString)));
    QObject::connect(graph_mgr, SIGNAL(setGUIInfo2(QString)), gui, SLOT(setInfo2(QString)));
}

/** On program startup:
 * Create 
 * - a Qt Application 
 * - an Object representing the ROS Node and its callback loop, 
 * - an OpenNIListener, setting up subscribers and callbacks for various formats of RGBD data
 * - a GraphManager, getting Nodes constructed from the RGBD data
 * - A Class providing a service call interface for ROS
 * - If applicable also a GUI
 * - let the above communicate internally via QT Signals, where communcication needs to be across threads or if the communication is conditional on the ROS node's parameterization.
 */
int main(int argc, char** argv)
{
  setlocale(LC_NUMERIC,"C");//Avoid expecting german decimal separators in launch files

  //create thread object, to run the ros event processing loop in parallel to the qt loop
  QtROS qtRos(argc, argv, "rgbdslam"); //ros node name & namespace

  //Depending an use_gui on the Parameter Server, a gui- or a headless application is used
  QApplication app(argc, argv, ParameterServer::instance()->get<bool>("use_gui")); 

  GraphManager graph_mgr(qtRos.getNodeHandle());

  //Instantiate the kinect image listener
  OpenNIListener listener(qtRos.getNodeHandle(), &graph_mgr);
  QObject::connect(&listener, SIGNAL(bagFinished()), &qtRos, SLOT(quitNow()));


  Graphical_UI* gui = NULL;
        if (app.type() == QApplication::GuiClient){
      gui = new Graphical_UI();
      gui->show();
      gui_connections(gui, &graph_mgr, &listener);
      ui_connections(gui, &graph_mgr, &listener);//common connections for the user interfaces
  } else {
      ROS_WARN("Running without graphical user interface! See README for how to interact with RGBDSLAM.");
  }
  //Create Ros service interface with or without gui
  RosUi ui("rgbdslam"); //ui namespace for service calls
  ui_connections(&ui, &graph_mgr, &listener);//common connections for the user interfaces

  //If one thread receives a exit signal from the user, signal the other thread to quit too
  QObject::connect(&app, SIGNAL(aboutToQuit()), &qtRos, SLOT(quitNow()));
  QObject::connect(&qtRos, SIGNAL(rosQuits()), &app, SLOT(quit()));
  QObject::connect(&listener, SIGNAL(bagFinished()), &app, SLOT(quit()));



#ifdef USE_ICP_BIN
  ROS_INFO_COND(ParameterServer::instance()->get<bool>("use_icp"), "ICP activated via external binary");
#endif
#ifdef USE_ICP_CODE
  ROS_INFO_COND(ParameterServer::instance()->get<bool>("use_icp"), "ICP activated via linked library");
#endif

  qtRos.start();// Run main loop.
  app.exec();
  if(ros::ok()) ros::shutdown();//If not yet done through the qt connection
  ros::waitForShutdown(); //not sure if necessary. 
  //delete gui;
}


