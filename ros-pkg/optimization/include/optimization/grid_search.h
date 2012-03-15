#ifndef GRID_SEARCH_H
#define GRID_SEARCH_H

#include <optimization/optimization.h>

class GridSearch
{
public:
  bool verbose_;
  ScalarFunction::Ptr objective_;
  //! max_resolutions_(i) is the starting step size for variable i.
  //! Step size decreases with each scaling.
  Eigen::ArrayXd max_resolutions_;
  //! grid_radii_(i) is how many steps to search in each direction for variable i.
  Eigen::ArrayXi grid_radii_;
  //! scale_factors_(i) is the factor to apply to the search resolution for variable i.
  Eigen::ArrayXd scale_factors_;
  //! Number of times to scale down the search.
  int num_scalings_;

  std::vector<Eigen::ArrayXd> history_;

  GridSearch(int num_variables);
  Eigen::ArrayXd search(const Eigen::ArrayXd& x);

protected:
  Eigen::ArrayXd x_;
  //! Current stepsizes for all variables.
  Eigen::ArrayXd res_;
  double best_obj_;
};


#endif // GRID_SEARCH_H
