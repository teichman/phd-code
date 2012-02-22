#ifndef L1TREE_H
#define L1TREE_H

//#include <multibooster/multibooster.h>
#include <Eigen/Eigen>
#include <Eigen/SVD>

class WeakClassifier;
class MultiBoosterDataset;

class L1TreeParams
{
public:
  int max_depth_;
  double min_expected_num_eliminated_;
  size_t min_wcs_;
};

class L1Tree
{
public:
  //! User-friendly constructor.
  L1Tree(const std::vector<WeakClassifier*>& pwcs, const L1TreeParams& params, MultiBoosterDataset* mbd, size_t dspace);
  //! Deletes the entire tree recursively.
  ~L1Tree();
  //! Gets activated weak classifiers using the tree.
  void query(const Eigen::VectorXf& point, std::vector<WeakClassifier*>* activated, int* num_evaluated) const;
  
private:
  L1Tree* parent_;
  Eigen::VectorXf projector_;
  //! A point with projection less than this is on the left; otherwise it's on the right.
  float threshold_;
  //! All the weak classifiers that must be considered at this node.
  std::vector<WeakClassifier*> pwcs_;
  L1Tree* rchild_;
  L1Tree* lchild_;
  L1TreeParams params_;
  int depth_;
  MultiBoosterDataset* mbd_;
  size_t dspace_;

  //! Recursive growth constructor.
  L1Tree(L1Tree* parent, const std::vector<WeakClassifier*>& pwcs);
  void growTree();
  double setThreshold(std::vector<WeakClassifier*>* left, std::vector<WeakClassifier*>* right);
  bool isLeaf() const;
  void doneGrowing() const;
  void getProjectorsOfParentsWithSameWCs(std::vector<Eigen::VectorXf>* projectors) const;
};

//! instances has columns of vectors.
//! Returns a matrix with columns of principal components.
Eigen::MatrixXd pca(const Eigen::MatrixXd& instances, int num = -1);
Eigen::VectorXd createProjectionDirection(const Eigen::VectorXd& principal_component);
Eigen::VectorXd firstPrincipalComponent(const Eigen::MatrixXd& instances);
Eigen::VectorXd randomVector(int num);
bool isDuplicateProjector(const Eigen::VectorXf& projector, const std::vector<Eigen::VectorXf>& others);
Eigen::VectorXd bruteForceProjectionSearch(const Eigen::MatrixXd& instances);

#endif // L1TREE_H
