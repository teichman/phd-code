#ifndef CONNECTED_COMPONENTS_H
#define CONNECTED_COMPONENTS_H

#include <vector>
#include <map>
#include <Eigen/Eigen>


class Graph_t
{
public:
  Graph_t();
  
  void addNode(size_t id);
  void addEdge(size_t id0, size_t id1);
  bool getNeighbors(size_t id, std::vector<size_t> &neighbors);
  void getSubgraphs(std::vector<Graph_t> &subgraphs, size_t min_size=2);
  
  typedef size_t Node_t;
  struct Edge_t
  {
    Node_t id0;
    Node_t id1;
  };
  
  // Stores actual node
  std::vector<Node_t> nodes_;
  // Edge by id
  std::vector<Edge_t> edges_;
protected:
  
  int getConnectedNodes(Node_t root, std::vector<Node_t> &connected);

  // Node by ID
  std::map<Node_t, size_t> id_to_idx_;
  // Cache of neighboring IDs
  std::map<Node_t, std::vector<Node_t> > neighbor_cache_;
  std::map<Node_t, std::vector<Node_t> > outgoing_neighbor_cache_;
};

#endif //CONNECTED_COMPONENTS_H
