#include <graphcuts/model.h>

using namespace Eigen;
using namespace std;

namespace graphcuts
{

  Model::Model()
  {
  }

  Model::Model(const Eigen::VectorXd& epot_weights,
	       const Eigen::VectorXd& npot_weights,
	       const NameMapping& epot_names,
	       const NameMapping& npot_names) :
    epot_weights_(epot_weights),
    npot_weights_(npot_weights),
    epot_names_(epot_names),
    npot_names_(npot_names)
  {
  }
  
  void Model::serialize(std::ostream& out) const
  {
    ROS_ASSERT(npot_weights_.rows() == (int)npot_names_.size());
    ROS_ASSERT(epot_weights_.rows() == (int)epot_names_.size());

    out << "Graphcuts Model v0.1" << endl;
    out << "== " << epot_weights_.rows() << " edge potential weights == " << endl;
    for(int i = 0; i < epot_weights_.rows(); ++i)
      out << epot_names_.toName(i) << "\t" << setprecision(16) << epot_weights_(i) << endl;
    out << "== " << npot_weights_.rows() << " node potential weights == " << endl;
    for(int i = 0; i < npot_weights_.rows(); ++i)
      out << npot_names_.toName(i) << "\t" << setprecision(16) << npot_weights_(i) << endl;
  }
  
  void Model::deserialize(std::istream& in)
  {
    string buf;
    getline(in, buf);
    ROS_ASSERT(buf.compare("Graphcuts Model v0.1") == 0);

    in >> buf;
    int num_edge;
    in >> num_edge;
    getline(in, buf);
    vector<string> epot_names(num_edge);
    epot_weights_ = VectorXd(num_edge);
    for(int i = 0; i < num_edge; ++i) {
      in >> epot_names[i];
      in >> epot_weights_(i);
    }
    epot_names_ = NameMapping();
    epot_names_.addNames(epot_names);

    in >> buf;
    int num_node;
    in >> num_node;
    getline(in, buf);
    vector<string> npot_names(num_node);
    npot_weights_ = VectorXd(num_node);
    for(int i = 0; i < num_node; ++i) {
      in >> npot_names[i];
      in >> npot_weights_(i);
    }
    npot_names_ = NameMapping();
    npot_names_.addNames(npot_names);
  }

  double Model::score(const VectorXd& psi) const
  {
    ROS_ASSERT(psi.rows() == epot_weights_.rows() + npot_weights_.rows());
    double val = psi.head(epot_weights_.rows()).dot(epot_weights_);
    val += psi.tail(npot_weights_.rows()).dot(npot_weights_);
    return val;
  }

  Eigen::VectorXd Model::concatenate() const
  {
    VectorXd weights(size());
    weights.head(epot_weights_.rows()) = epot_weights_;
    weights.tail(npot_weights_.rows()) = npot_weights_;
    return weights;
  }
  
}
