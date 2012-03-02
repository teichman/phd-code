#include <graphcuts/structural_svm.h>

using namespace std;
namespace bfs = boost::filesystem;
namespace bpt = boost::posix_time;
using namespace Eigen;
using namespace pipeline2;

namespace graphcuts
{

  StructuralSVM::StructuralSVM(double c,
			       double precision,
			       int num_threads,
			       int debug_level) :
    c_(c),
    precision_(precision),
    num_threads_(num_threads),
    debug_level_(debug_level)
  {
  }
  
  Eigen::VectorXd StructuralSVM::train(const std::vector<PotentialsCache::Ptr>& caches,
				       const std::vector<VecXiPtr>& labels) const
						   
  {
    ROS_ASSERT(caches.size() == labels.size());
    ROS_ASSERT(!caches.empty());

    // Start slightly away from the boundary.
    VectorXd weights = 0.001 * VectorXd::Ones(caches[0]->getNumPotentials()); 
    double slack = 0.001;
    int iter = 0;
    int num_edge_weights = caches[0]->getNumEdgePotentials();
    double best_loss = std::numeric_limits<double>::max();
    VectorXd best_weights = weights;
    vector<Constraint> constraints;
    
    while(true) {
      ROS_DEBUG_STREAM("==================== Starting iteration " << iter << flush);
      VectorXd prev_weights = weights;
      double loss = 0;

      // -- Compute the most violating labeling for each framecache.
      vector<ComputeNode*> nodes(caches.size(), NULL);
      for(size_t i = 0; i < caches.size(); ++i)
	nodes[i] = new ConstraintGenerator(weights, caches[i], labels[i]);
      Pipeline2 pl(num_threads_, nodes);
      HighResTimer hrt("Computing most violating constraints");
      hrt.start();
      pl.compute();
      hrt.stop();

      // -- Compute the one new constraint.
      Constraint c;
      c.loss_ = 0;
      c.dpsi_ = VectorXd::Zero(weights.rows());
      for(size_t i = 0; i < nodes.size(); ++i) {
	ConstraintGenerator& cg = *(ConstraintGenerator*)nodes[i];
	ROS_ASSERT(cg.hamming_loss_ >= 0);  // Make sure all nodes computed.
	loss += cg.hamming_loss_ / (double)caches.size();
	c.loss_ += cg.con_.loss_ / (double)caches.size(); // mean 0-1 loss
	c.dpsi_ += cg.con_.dpsi_ / (double)caches.size(); // mean dpsi
      }
      constraints.push_back(c);
      ROS_DEBUG_STREAM("Mean loss: " << loss);
      if(loss < best_loss) {
	ROS_DEBUG_STREAM("Best normalized loss so far (previous best: " << best_loss << ").");
	best_loss = loss;
	best_weights = weights;
      }
      
      // -- Check if we're done.
      double margin = weights.dot(c.dpsi_);
      ROS_DEBUG_STREAM("loss - margin: " << c.loss_ - margin);
      ROS_DEBUG_STREAM("Slack: " << slack);
      if(c.loss_ - margin <= slack + 1e-6) {
	ROS_DEBUG_STREAM("Breaking because the newly added constraint is already satisfied.");
	break;
      }
      
      // -- Run the solver.
      ROS_DEBUG_STREAM("Total constraints: " << constraints.size());
      hrt.reset("Learning new weights");
      hrt.start();
      updateWeights(constraints,
		    num_edge_weights,
		    &weights, &slack);
      hrt.stop();
      ROS_DEBUG_STREAM(hrt.report() << flush);
      cout << weights.transpose() << endl;

      ++iter;
    }

    return weights;
  }

  double StructuralSVM::updateWeights(const std::vector<Constraint>& constraints,
				      int num_edge_weights,
				      Eigen::VectorXd* weights,
				      double* slack) const
  {
    // -- Warm start from previous solution.
    VectorXd x = VectorXd::Zero(weights->rows() + 1);
    x.head(weights->rows()) = *weights;
    x(x.rows() - 1) = *slack;

    // ... but make sure it's a feasible starting point.
    for(size_t i = 0; i < constraints.size(); ++i) {
      const Constraint& c = constraints[i];
      double& sl = x.coeffRef(weights->rows());
      if(weights->dot(c.dpsi_) < c.loss_ - sl) { 
	// Strict equality won't work with this solver,
	// so add a bit to the minimum amount of slack.
	sl = c.loss_ - weights->dot(c.dpsi_) + 1.0;
      }
    }
    
    // -- Generate the objective function and gradient.
    SMPtr A(new Eigen::SparseMatrix<double>(x.rows(), x.rows()));
    for(int i = 0; i < weights->rows(); ++i) {
      A->startVec(i);
      A->insertBack(i, i) = 1.0;
    }
    A->finalize();

    SVPtr b(new SparseVector<double>(x.rows()));
    b->startVec(0);
    for(int i = 0; i < x.rows(); ++i)
      if(i >= weights->rows())
	b->insertBack(i) = c_;
    b->finalize();

    SparseQuadraticFunction::Ptr objective(new SparseQuadraticFunction(A, b, 0));
    SparseLinearFunction::Ptr gradient(new SparseLinearFunction(A, b));
    
    // -- Set up the optimizer.
    double initial_mu = 1.0;
    double alpha = 0.3;
    double beta = 0.5;
    int max_num_iters = 0;
    double stepsize = 1;
    int restart = 0;
    ROS_DEBUG_STREAM("Solving with tolerance " << precision_);
    NesterovInteriorPointSolver nips(objective, gradient,
				     initial_mu, precision_, alpha, beta,
				     max_num_iters, stepsize, restart, debug_level_);

    // -- Generate the margin constraints.
    for(size_t i = 0; i < constraints.size(); ++i) {
      SMPtr A(new Eigen::SparseMatrix<double>(x.rows(), x.rows()));
      A->finalize();

      SVPtr b(new Eigen::SparseVector<double>(x.rows()));
      b->startVec(0);
      for(int j = 0; j < weights->rows(); ++j)
	b->insertBack(j) = -constraints[i].dpsi_(j);
      b->insertBack(weights->rows()) = -1;

      SparseQuadraticFunction::Ptr c(new SparseQuadraticFunction(A, b, constraints[i].loss_));
      SparseLinearFunction::Ptr gc(new SparseLinearFunction(A, b));
      nips.addConstraint(c, gc);
    }
    
    // -- Generate the non-negativity constraints for the edge weights
    //    and the slack variables.
    for(int i = 0; i < x.rows(); ++i) {
      if(i < num_edge_weights || i >= weights->rows()) { 
	SMPtr A(new Eigen::SparseMatrix<double>(x.rows(), x.rows()));
	A->finalize();
	
	SVPtr b(new Eigen::SparseVector<double>(x.rows()));
	b->startVec(0);
	b->insertBack(i) = -1.0;
	b->finalize();

	SparseQuadraticFunction::Ptr c(new SparseQuadraticFunction(A, b, 0));
	SparseLinearFunction::Ptr gc(new SparseLinearFunction(A, b));
	nips.addConstraint(c, gc);
      }
    }

    // -- Solve.
    long int ns = 0;
    VectorXd xstar = nips.solve(x, &ns);
    ROS_DEBUG_STREAM("Solving with Nesterov took " << ns << " steps, total.");
    *slack = xstar(xstar.rows() - 1);
    *weights = xstar.head(weights->rows());
    
    return objective->eval(xstar);
  }  
  
  ConstraintGenerator::ConstraintGenerator(const Eigen::VectorXd& weights,
					   PotentialsCache::ConstPtr cache,
					   VecXiConstPtr labels) :
    ComputeNode(),
    hamming_loss_(-1),
    weights_(weights),
    cache_(cache),
    labels_(labels)
  {
  }
     
  void ConstraintGenerator::_compute()
  {
    MaxflowInference mfi(weights_);
    VecXi seg;
    mfi.segment(cache_, &seg);
    hamming_loss_ = hammingLoss(*labels_, seg);
    double zero_one_loss = zeroOneLoss(*labels_, seg);

    VectorXd psi_gt = cache_->psi(*labels_);
    double gt_score = weights_.dot(psi_gt);
    VectorXd psi_pred = cache_->psi(seg);
    double pred_score = weights_.dot(psi_pred);
    VectorXd dpsi = psi_gt - psi_pred;

    // The segmentation provided by MaxflowInference should have the
    // lowest score possible for these weights.
    ROS_FATAL_STREAM_COND(pred_score < gt_score,
			  "Prediction score (" << pred_score
			  << ") is less than ground truth score ("
			  << gt_score << ")!");
    
    con_.dpsi_ = dpsi;
    con_.loss_ = zero_one_loss;
  }

  void ConstraintGenerator::_flush()
  {
    con_ = Constraint();
    hamming_loss_ = -1;
  }

  double hammingLoss(const Eigen::VectorXi& label,
		     const Eigen::VectorXi& pred)
  {
    ROS_ASSERT(label.rows() == pred.rows());
    ROS_ASSERT(label.cols() == pred.cols());

    double loss = 0;
    for(int i = 0; i < label.rows(); ++i)
      if(label(i) != pred(i))
	++loss;

    return loss;
  }

  double zeroOneLoss(const Eigen::VectorXi& label,
		     const Eigen::VectorXi& pred)
  {
    ROS_ASSERT(label.rows() == pred.rows());
    ROS_ASSERT(label.cols() == pred.cols());

    for(int i = 0; i < label.rows(); ++i)
      if(label(i) != pred(i))
	return 1.0;

    return 0.0;
  }
  
} // namespace dst

