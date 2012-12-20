#include <asp/asp.h>

using namespace std;

void NodePotentialAggregator::setWeights(const Model& model)
{
  const vector<const Outlet*>& inputs = inputs_["NodePotentials"];
  ROS_ASSERT((size_t)model.nweights_.rows() == inputs.size());
  ROS_ASSERT((size_t)model.nweights_.rows() == model.nameMapping("nmap").size());
  for(size_t i = 0; i < inputs.size(); ++i)
    ROS_ASSERT(inputs[i]->getPod()->getName() == model.nameMapping("nmap").toName(i));
  
  nweights_ = model.nweights_;
}

void NodePotentialAggregator::fillModel(Model* model) const
{
  const vector<const Outlet*>& inputs = inputs_["NodePotentials"];
  ROS_ASSERT((size_t)nweights_.rows() == inputs.size());
  NameMapping nmap;
  for(size_t i = 0; i < inputs.size(); ++i) {
    nmap.addName(inputs[i]->getPod()->getName());
  }
  model->applyNameMapping("nmap", nmap);
  model->nweights_ = nweights_;
}

void NodePotentialAggregator::compute()
{
  vector<MatrixXfConstPtr> npots;
  pull("NodePotentials", &npots);

  ROS_ASSERT((size_t)nweights_.rows() == npots.size());
}

