#ifndef GRID_SEARCH_H
#define GRID_SEARCH_H

#include <optimization/optimization.h>

class GridSearchViewHandler
{
public:
  virtual void handleGridSearchUpdate(const Eigen::ArrayXd& x, double objective) = 0;
};

class GridSearch
{
public:
  bool verbose_;
  ScalarFunction::Ptr objective_;
  //! Number of times to scale down the search.
  int num_scalings_;
  //! max_resolutions_(i) is the starting step size for variable i.
  //! Step size decreases with each scaling.
  Eigen::ArrayXd max_resolutions_;
  //! grid_radii_(i) is how many steps to search in each direction for variable i.
  Eigen::ArrayXi grid_radii_;
  //! scale_factors_(i) is the factor to apply to the search resolution for variable i.
  Eigen::ArrayXd scale_factors_;
  //! Which variables to optimize jointly.
  //! By default, this is 0 1 2 3 ... num_vars-1
  //! Each variable gets a coupling ids.  They must start with zero
  //! and not be larger than num_vars - 1.
  Eigen::ArrayXd couplings_;
  GridSearchViewHandler* view_handler_;
  
  std::vector<Eigen::ArrayXd> history_;

  GridSearch(int num_variables);
  Eigen::ArrayXd search(const Eigen::ArrayXd& x);

protected:
  Eigen::ArrayXd x_;
  //! Current stepsizes for all variables.
  Eigen::ArrayXd res_;
  double best_obj_;

  void appendVariations(int id, const Eigen::ArrayXd& orig,
			std::vector<Eigen::ArrayXd>* xs) const;
  void makeGrid(const std::vector<int>& variables,
		std::vector<Eigen::ArrayXd>* xs) const;
  
};

#endif // GRID_SEARCH_H
