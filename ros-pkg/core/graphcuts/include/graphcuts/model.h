#ifndef GRAPHCUTS_MODEL_H
#define GRAPHCUTS_MODEL_H

#include <serializable/serializable.h>
#include <name_mapping2/name_mapping2.h>
#include <eigen_extensions/eigen_extensions.h>

namespace graphcuts
{

  class Model : public Serializable
  {
  public:
    Eigen::VectorXd epot_weights_;
    Eigen::VectorXd npot_weights_;
    NameMapping2 epot_names_;
    NameMapping2 npot_names_;

    Model();
    Model(const Eigen::VectorXd& epot_weights,
	  const Eigen::VectorXd& npot_weights,
	  const NameMapping2& epot_names = NameMapping2(),
	  const NameMapping2& npot_names = NameMapping2());
    void serialize(std::ostream& out) const;
    void deserialize(std::istream& in);
    //! Edge, then node potentials.
    double score(const Eigen::VectorXd& psi) const;
    //double score(const VectorXd& edge_psi, const Eigen::VectorXd& node_psi) const;
    int size() const { return epot_weights_.rows() + npot_weights_.rows(); }
    Eigen::VectorXd concatenate() const;
  };
  
}

#endif // GRAPHCUTS_MODEL_H
