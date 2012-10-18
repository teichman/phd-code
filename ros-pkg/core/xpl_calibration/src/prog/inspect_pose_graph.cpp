
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <gperftools/profiler.h>
#include <pose_graph_slam/pose_graph_slam.h>
#include <rgbd_sequence/stream_sequence.h>
#include <rgbd_sequence/rgbd_sequence.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <xpl_calibration/utility_functions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <xpl_calibration/trajectory.h>
#include <xpl_calibration/primesense_slam.h>

using namespace std;
using namespace g2o;
using namespace rgbd;


typedef pcl::PointXYZ PointBW_t;
typedef pcl::PointCloud<PointBW_t> CloudBW_t;
typedef pcl::PointXYZRGB Point_t;
typedef pcl::PointCloud<Point_t> Cloud_t;

int main(int argc, char** argv)
{
  namespace bpo = boost::program_options;
  namespace bfs = boost::filesystem;
  bpo::options_description opts_desc("Allowed options");
  bpo::positional_options_description p;

  opts_desc.add_options()
    ("help,h", "produce help message")
    ("sseq", bpo::value<string>()->required(), "StreamSequence, i.e. asus data.")
    ("posegraph", bpo::value<string>()->required(), "Input path for the complete pose graph.")
    ("otraj", bpo::value<string>(), "Out path for the final trajectory.")
    ("opcd", bpo::value<string>(), "Out path for the final map pcd.")
    ("visualize", "Visualize incrementally removed edges")
    ;

  p.add("sseq",1).add("posegraph", 1);
  
  bpo::variables_map opts;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), opts);
  bool badargs = false;
  try { bpo::notify(opts); }
  catch(...) { badargs = true; }
  if(opts.count("help") || badargs) {
    cout << "Usage: " << bfs::basename(argv[0]) << " SSEQ POSEGRAPH [OPTS]" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }
  string otraj = opts.count("otraj") ? opts["otraj"].as<string>() : "";
  string opcd = opts.count("opcd") ? opts["opcd"].as<string>() : "";

  cout << "Using " << opts["sseq"].as<string>() << endl;
  cout << "Using pose graph " << opts["posegraph"].as<string>() << endl;
  bool visualize = opts.count("visualize");
  StreamSequence::Ptr sseq(new StreamSequence);
  sseq->load(opts["sseq"].as<string>());
  PoseGraphSlam slam;
  slam.load(opts["posegraph"].as<string>());

  pcl::visualization::PCLVisualizer vis;
  pcl::visualization::PCLVisualizer map_vis("map");
  //Build the map
  slam.solve();
  // Prune disconnected components
  vector<size_t> nodes_with_edges;
  for(size_t i = 0; i < slam.numNodes(); i++)
  {
    if(slam.numEdges(i) > 0) nodes_with_edges.push_back(i);
  }
  vector<vector<size_t> > edges(slam.numNodes()); //edges[i] = edges contected to i;
  for(size_t i = 0; i < slam.edges_.size(); i++)
  {
    const EdgeStruct &e = slam.edges_[i];
    edges[e.idx0].push_back(e.idx1);
    edges[e.idx1].push_back(e.idx0);
  }
  vector<bool> visited(slam.numNodes(), false);
  vector<size_t> tovisit;
  tovisit.push_back(nodes_with_edges[0]);
  while(tovisit.size() > 0)
  {
    size_t curnode = tovisit[tovisit.size()-1];
    tovisit.pop_back();
    if(visited[curnode]) continue;
    visited[curnode] = true;
    for(size_t i = 0; i < edges[curnode].size(); i++)
    {
      size_t neighbor = edges[curnode][i];
      if(!visited[neighbor])
        tovisit.push_back(neighbor);
    }
  }
  for(size_t i = 0; i < nodes_with_edges.size(); i++)
  {
    if(!visited[nodes_with_edges[i]]) cout << "Unvisited: " << nodes_with_edges[i] << endl;
  }
  while(true)
  {
    if(visualize)
    {
      vis.removeAllPointClouds();
      vis.removeAllShapes();
      map_vis.removeAllPointClouds();
      map_vis.removeAllShapes();
    }
    // See which is the max violated node
    slam.solve();
    vector<double> errors(slam.edges_.size());
    for(size_t i = 0; i < slam.edges_.size(); i++)
    {
      const EdgeStruct &e = slam.edges_[i];
      //const Eigen::Affine3d &predicted_trans = slam.transform(e.idx0)*e.transform;
      //Eigen::Affine3d map_trans = slam.transform(e.idx1);
      //errors[i] = (map_trans.matrix() - predicted_trans.matrix()).norm();
      const Eigen::Affine3d &pairwise_trans = e.transform;
      Eigen::Affine3d predicted_pairwise_trans = slam.transform(e.idx0).inverse()*slam.transform(e.idx1);
      errors[i] = (pairwise_trans.matrix() - predicted_pairwise_trans.matrix()).norm();
    }
    vector<size_t> idxs;
    vector<double> errors_desc;
    sortv(errors, errors_desc, idxs, DESCENDING);
    for(size_t i = 0; i < errors_desc.size(); i++)
    {
      const EdgeStruct &e = slam.edges_[idxs[i]];
      cout << "Edge " << e.idx1 << "->" << e.idx0 << ": " << errors_desc[i] << endl;
      if(i > 25)
        break;
    }
    //Draw nodes in a circle
    size_t num_nodes = slam.numNodes();
    float dtheta = (7*M_PI/4) / num_nodes;
    CloudBW_t::Ptr nodes(new CloudBW_t);
    CloudBW_t::Ptr active_nodes(new CloudBW_t);
    float r = 5;
    for(size_t i = 0; i < num_nodes; i++)
    {
      float x = r*cos(dtheta*i);
      float y = r*sin(dtheta*i);
      float z = 0.5;
      nodes->points.push_back(PointBW_t(x, y, z));
      if(slam.numEdges(i) > 0)
        active_nodes->points.push_back(PointBW_t(x,y,z));
    }
    if(visualize)
    {
      //Draw all nodes in blue
      pcl::visualization::PointCloudColorHandlerCustom<PointBW_t> handler(nodes, 0, 0, 255);
      vis.addPointCloud(nodes, handler, "nodes");
      vis.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "nodes");
      //Draw active (connected) nodes in green
      int pt_r = 2;
      pcl::visualization::PointCloudColorHandlerCustom<PointBW_t> active_handler(active_nodes, 0, 255, 0);
      vis.addPointCloud(active_nodes, active_handler, "active_nodes");
      vis.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pt_r, "active_nodes");
      //Highlight the start point
      vis.addSphere(nodes->points[0], 0.05, 0, 255, 255, "start");
      //Draw all edges
      //Color by max violation, where gray is nothing and red is 0.2
      for(size_t i = 0; i < slam.edges_.size(); i++)
      {
        const EdgeStruct& e = slam.edges_[i];
        const PointBW_t &start = nodes->points[e.idx0];
        const PointBW_t &end = nodes->points[e.idx1];
        ostringstream oss;
        oss << "edge " << i;
        vis.addLine(start, end, (0.9*errors[i]/0.2)+0.1, 0.1, 0.1, oss.str());
      }
      vis.spin();
    }
    //Visualize the worst edge
    if(errors_desc[0] < 0.1)
    {
      cout << "Max error " << errors_desc[0] << "< threshold " << 0.1 << endl;
      break;
    }
    const EdgeStruct &worst = slam.edges_[idxs[0]];

    Cloud_t::Ptr prev = sseq->getCloud(worst.idx0);
    Cloud_t::Ptr curr = sseq->getCloud(worst.idx1);
    Cloud_t::Ptr curr_trans(new Cloud_t);
    pcl::transformPointCloud(*curr, *curr_trans, worst.transform.cast<float>());
    if(visualize)
    {
      map_vis.addPointCloud(prev, "prev");
      map_vis.addPointCloud(curr_trans, "curr");
      map_vis.spin();
    }
    //Remove it
    slam.removeEdge(idxs[0]);
    slam.solve();
    cout << "Removed!" << endl;
  }
  //Build the map again
  cout << "Rebuilding map" << endl;
  pcl::VoxelGrid<Point_t> vg;
  vg.setLeafSize(0.02, 0.02, 0.02);
  vector<vector<int> > subgraphs; slam.getSubgraphs(subgraphs);
  
  for(size_t i = 0; i < subgraphs.size(); i++)
  {
    // Make map
    Cloud::Ptr map(new Cloud);
    const vector<int> &subgraph = subgraphs[i];
    cout << "Building map " << i+1 << "/" << subgraphs.size() << endl;
    for(size_t j = 0; j < subgraph.size(); j++)
    {
      int node = subgraph[j];
      //Get the cloud
      Cloud::Ptr curr_pcd = sseq->getCloud(node);
      zthresh(curr_pcd, MAX_RANGE_MAP);
      Cloud::Ptr curr_pcd_transformed(new Cloud);
      //Transform it
      Eigen::Affine3d trans = slam.transform(node);
      pcl::transformPointCloud(*curr_pcd, *curr_pcd_transformed, trans.cast<float>());
      *map += *curr_pcd_transformed;
      //Filter it
      Cloud::Ptr vis(new Cloud);
      vg.setInputCloud(map);
      vg.filter(*vis);
      *map = *vis;
    }
    cout << "Done" << endl;
    // Visualize it
    if(visualize)
    {

      if(!map_vis.addPointCloud(map, "map"))
        map_vis.updatePointCloud(map, "map");
    }
    // Save it
    if(opcd != "")
    {
      string pcdname;
      if(i == 0) pcdname = opcd;
      else
      {
        string basename = opcd.substr(0, opcd.length() - 4);
        ostringstream oss;
        oss << basename << "_" << i << ".pcd";
        pcdname = oss.str();
      }
      
      pcl::io::savePCDFileBinary(pcdname, *map);
      cout << "Saved final map to " << pcdname << endl;
    }
    // Make trajectory
    if(otraj == "") continue;
    Trajectory traj;
    traj.resize(slam.numNodes());
    for(size_t j = 0; j < subgraph.size(); j++)
    {
      int node = subgraph[j];
      traj.set(node, slam.transform(node));
    }
    string trajname;
    if(i == 0) trajname = otraj;
    else
    {
      string basename = otraj.substr(0, otraj.length() - 5);
      ostringstream oss;
      oss << basename << "_" << i << ".traj";
      trajname = oss.str();
    }
    traj.save(trajname);
    cout << "Saved trajectory to " << trajname << endl;
  }
//  for(size_t i = 0; i < slam.numNodes(); i++)
//  {
//    if(slam.numEdges(i)==0) continue;
//    if(!visited[i]) continue;
//    //Get the cloud
//    Cloud::Ptr curr_pcd = sseq->getCloud(i);
//    zthresh(curr_pcd, MAX_RANGE_MAP);
//    Cloud::Ptr curr_pcd_transformed(new Cloud);
//    //Transform it
//    Eigen::Affine3d trans = slam.transform(i);
//    pcl::transformPointCloud(*curr_pcd, *curr_pcd_transformed, trans.cast<float>());
//    *map += *curr_pcd_transformed;
//    //Filter it
//    Cloud::Ptr vis(new Cloud);
//    vg.setInputCloud(map);
//    vg.filter(*vis);
//    *map = *vis;
//  }
//  if(visualize)
//  {
//    map_vis.addPointCloud(map, "map");
//    cout << "Showing map" << endl;
//    map_vis.spin();
//  }
//  if(opcd != "")
//  {
//    pcl::io::savePCDFileBinary(opcd, *map);
//    cout << "Saved final map to " << opcd << endl;
//  }
//  if(otraj != "")
//  {
//    Trajectory traj;
//    traj.resize(slam.numNodes());
//    for(size_t i = 0; i < slam.numNodes(); ++i)
//    {
//      if(slam.numEdges(i) == 0) continue;
//      if(!visited[i]) continue;
//      traj.set(i, slam.transform(i));
//    }
//    traj.save(otraj);
//    cout << "Saved trajectory to " << otraj << endl;
//  }


  return 0;
}
