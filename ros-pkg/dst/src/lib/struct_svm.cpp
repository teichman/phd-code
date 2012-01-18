#include <dst/struct_svm.h>

#define NUM_THREADS (getenv("NUM_THREADS") ? atoi(getenv("NUM_THREADS")) : 1)
#define PRINT_MATRICES (getenv("PRINT_MATRICES"))
#define SUPPRESS_VIS (getenv("SUPPRESS_VIS"))

using namespace std;
namespace bfs = boost::filesystem;
namespace bpt = boost::posix_time;
using namespace Eigen;
using namespace qpOASES;

namespace dst
{

  StructSVM::StructSVM(double c,
		       double precision,
		       bool margin_rescaling) :
    c_(c),
    precision_(precision),
    margin_rescaling_(margin_rescaling)
  {
  }
  
  void StructSVM::loadSequences(const std::string& path)
  {
    dst::loadSequences(path, &sequences_);
  }
  
  Eigen::VectorXd StructSVM::trainRobust2()
  {
    Eigen::VectorXd weights = VectorXd::Zero(getNumWeights());
    Eigen::VectorXd prev_weights = weights;
    int iter = 0;
    while(true) {
      ROS_DEBUG_STREAM("============================================================");
      ROS_DEBUG_STREAM("=          Starting outer iteration " << iter);
      ROS_DEBUG_STREAM("============================================================");
      int total_segmented_frames = 0;
      vector<FramePotentialsCache::Ptr> caches;
      vector<cv::Mat1b> labels;
      
      // -- Build the training set in a way that will produce some non-optimal,
      //    realistic sets of potentials.
      for(size_t i = 0; i < sequences_.size(); ++i) {
	KinectSequence::Ptr seq = sequences_[i];

	SegmentationPipeline sp(NUM_THREADS);
	sp.verbose_ = false;
	sp.setWeightsWithClipping(weights);
	if(i == 0)
	  ROS_DEBUG_STREAM("Weights: " << endl << sp.weightsStatus() << flush);
	cv::Mat1b working_img_seg(seq->seed_images_[0].size(), 127);
	cv::Mat1b empty_seed(seq->seed_images_[0].size(), 127);
	for(size_t j = 0; j < seq->segmentations_.size(); ++j) { 
	  // Segment using current weights.
	  KinectCloud::Ptr seg_pcd(new KinectCloud());  // Not used.
	  ++total_segmented_frames;
	  if(j == 0) {
	    sp.run(seq->seed_images_[j],
		   seq->images_[j],
		   seq->pointclouds_[j],
		   cv::Mat3b(),
		   cv::Mat1b(),
		   KinectCloud::Ptr(),
		   working_img_seg,
		   seg_pcd);
	    continue; // Don't add training examples for seed frames.
	  }
	  else {
	    sp.run(empty_seed,
		   seq->images_[j],
		   seq->pointclouds_[j],
		   seq->images_[j-1],
		   working_img_seg,
		   seq->pointclouds_[j-1],
		   working_img_seg,
		   seg_pcd);
	  }

	  labels.push_back(seq->segmentations_[j]);
	  caches.push_back(sp.getFrameCache());
	  
	  // If we're starting to do really poorly or if we're done,
	  // print out results and move on to the next sequence.
	  double hamming_loss = loss(seq->segmentations_[j], working_img_seg, true);
	  double max_hamming_loss = 30;  // TODO: parameterize
	  if(hamming_loss > max_hamming_loss) {
	    ROS_DEBUG_STREAM("Sequence " << i << ": Hamming loss for frame " << j + 1 << " / " << seq->images_.size()
			     << " is " << hamming_loss);
	    break;
	  }
	  else if(j == seq->images_.size() - 1)
	    ROS_DEBUG_STREAM("Sequence " << i << ": Completed without making serious errors.");
	}
      }

      // -- Run structural SVM.
      ROS_DEBUG_STREAM("Total number of segmented frames this outer iteration: " << total_segmented_frames << flush);
      ROS_DEBUG_STREAM("Learning new weights using " << caches.size() << " training examples.");
      prev_weights = weights;
      weights = runSolver(caches, labels);
      if(getenv("SAVE_ALL_WEIGHTS")) {
	ostringstream oss;
	oss << "weights" << setw(4) << setfill('0') << iter << ".eig.txt";
	eigen_extensions::saveASCII(weights, oss.str());
      }

      // -- Stop if we're done.
      if(iter > 0) {
	cout << "weights norm: " << weights.norm() << endl;
	ROS_ASSERT(fabs(weights.norm() - 1.0) < 1e-6);
	ROS_ASSERT(fabs(prev_weights.norm() - 1.0) < 1e-6);
      }
      if(fabs(acos(weights.dot(prev_weights))) < 1e-6) {
	ROS_DEBUG_STREAM("Breaking because weights have not changed much.");
	break;
      }
    
      ++iter;
    }

    return weights;
  }
  
  Eigen::VectorXd StructSVM::train()
  {
    // -- Build the training set assuming that segmentation is always correct.
    vector<FramePotentialsCache::Ptr> caches;
    vector<cv::Mat1b> labels;
    for(size_t i = 0; i < sequences_.size(); ++i) {
      KinectSequence::Ptr seq = sequences_[i];
      SegmentationPipeline sp(NUM_THREADS);
      PotentialsCache::Ptr cache = sp.cacheUnweightedPotentialsWithOracle(seq);

      for(size_t j = 0; j < cache->size(); ++j) { 
	caches.push_back(cache->framecaches_[j]);
	labels.push_back(seq->segmentations_[j]);
      }
    }

    // -- Run structural SVM.
    return runSolver(caches, labels);
  }
  
  Eigen::VectorXd StructSVM::runSolver(const std::vector<FramePotentialsCache::Ptr>& caches,
				       const std::vector<cv::Mat1b>& labels) const
  {
    ROS_ASSERT(caches.size() == labels.size());
    
    std::vector<Constraint> working;
    VectorXd weights = VectorXd::Zero(caches[0]->getNumWeights());
    VectorXd slacks = VectorXd::Zero(caches.size());
    int iter = 0;
    double best_hamming_loss = numeric_limits<double>::max();
    VectorXd best_weights = weights;
    
    while(true) {
      VectorXd prev_weights = weights;
      size_t prev_working_size = working.size();
      double hamming_loss = 0;

      ROS_DEBUG_STREAM("==================== Starting iteration " << iter << flush);
      
      // -- Add constraints for all training examples.
      for(size_t i = 0; i < caches.size(); ++i) {
	SegmentationPipeline sp(NUM_THREADS);
	sp.verbose_ = false;
	if(getenv("SAVE_ALL_WEIGHTS")) {
	  ostringstream oss;
	  oss << "weights" << setw(4) << setfill('0') << iter << ".eig.txt";
	  eigen_extensions::saveASCII(weights, oss.str());
	}
	sp.setWeightsWithClipping(weights);
	cv::Mat1b ymv = sp.findMostViolating(*caches[i], labels[i], margin_rescaling_);
	if(!SUPPRESS_VIS) {
	  cv::imshow("most violating", ymv);
	  cv::waitKey(5);
	}
	double l = loss(labels[i], ymv, margin_rescaling_);
	if(margin_rescaling_)
	  l /= (double)(ymv.rows * ymv.cols);
	hamming_loss += loss(labels[i], ymv, true) / (double)caches.size();
	if(l == 0) {
	  ROS_DEBUG_STREAM("Training example " << i << " is equal to ground truth.  Skipping.");
	  continue;
	}

	VectorXd psi_gt;
	double gt_score = caches[i]->computeScore(labels[i],
						  sp.getEdgeWeights(),
						  sp.getNodeWeights(),
						  &psi_gt);
	
	VectorXd psi_pred;
	double pred_score = caches[i]->computeScore(ymv,
						    sp.getEdgeWeights(),
						    sp.getNodeWeights(),
						    &psi_pred);
	VectorXd dpsi = psi_gt - psi_pred;

	// TODO: Don't add constraints that aren't active?
	// if(margin_rescaling_ && gt_score - pred_score - l + slacks(i) >= precision_) { 
	//   ROS_DEBUG_STREAM("Training example " << i << " is not violated sufficiently.  Skipping.");
	//   continue;
	// }

	
	// ymv should the best possible labeling for these weights.
	if(!margin_rescaling_ && pred_score < gt_score) { 
	  ROS_FATAL_STREAM("Prediction score (" << pred_score << ") is less than ground truth score (" << gt_score << ")!");
	  cv::imshow("ground truth", labels[i]);
	  cv::imshow("prediction", ymv);
	  cv::waitKey();
	  ROS_ASSERT(pred_score >= gt_score);  // This failed once.  Why?
	}
	
	Constraint c;
	c.tr_ex_id_ = i;
	c.dpsi_ = dpsi;
	c.loss_ = l;
	working.push_back(c);
      }
      
      // -- Keep track of the best weights vector.
      ROS_DEBUG_STREAM("Mean hamming loss: " << hamming_loss);
      ROS_DEBUG_STREAM("Best so far: " << best_hamming_loss);
      if(hamming_loss < best_hamming_loss) {
	ROS_DEBUG_STREAM("This iteration has best mean hamming loss so far: " << hamming_loss << " vs " << best_hamming_loss);
	best_hamming_loss = hamming_loss;
	best_weights = weights;
	
	string filename = "autosave_best_weights.eig.txt";
	ROS_DEBUG_STREAM("Saving best weights to " << filename);
	eigen_extensions::saveASCII(weights, filename);
      }
      
      // -- Check if we're done.
      if(working.size() == prev_working_size) {
	ROS_DEBUG_STREAM("Breaking due to lack of additions to working set.");
	break;
      }
      
      // -- Update the weights.
      bool feas = true;
      ROS_DEBUG_STREAM("Added " << working.size() - prev_working_size << " constraints.");
      ROS_DEBUG_STREAM("Total constraints: " << working.size());
      SegmentationPipeline sp(NUM_THREADS);
      int num_edge_weights = sp.getEdgeWeights().rows();
      
      // std::ostringstream msg;
      // const boost::posix_time::ptime now = bpt::second_clock::local_time();
      // bpt::time_facet* const f = new bpt::time_facet("%H-%M-%S");
      // msg.imbue(std::locale(msg.getloc(),f));
      // msg << now;
      
      bpt::ptime now = bpt::second_clock::local_time();
      ROS_DEBUG_STREAM("Starting to learn new weights at " << bpt::to_iso_extended_string(now));
      HighResTimer hrt("Learning new weights");
      hrt.start();
      updateWeights(working, caches.size(),
		    num_edge_weights,
		    &weights, &slacks, &feas);
      hrt.stop();
      ROS_DEBUG_STREAM(hrt.reportMinutes());
      ROS_ASSERT(fabs(weights.norm() - 1.0) < 1e-6);
      if(iter > 0) { 
	ROS_ASSERT(fabs(prev_weights.norm() - 1.0) < 1e-6);
	ROS_DEBUG_STREAM("angle change between current weights and old weights = " << acos(weights.dot(prev_weights)));
      }
      ROS_DEBUG_STREAM("new weights: ");
      ROS_DEBUG_STREAM(weights.transpose());

      // -- Check if we're done.
      // TODO: Make the problem never go infeasible.
      if(!feas) {
	ROS_WARN_STREAM("Breaking due to infeasibility.");
	break;
      }
      if(fabs(acos(weights.dot(prev_weights))) < 1e-6) {
	ROS_DEBUG_STREAM("Breaking because weights have not changed much.");
	break;
      }

      ++iter;
    }

    ROS_DEBUG_STREAM("Best hamming loss for this run: " << best_hamming_loss);
    return best_weights;
  }

  double* StructSVM::eigToQPO(const Eigen::MatrixXd& eig)
  {
    double* qpo = new double[eig.rows() * eig.cols()];
    for(int i = 0; i < eig.rows(); ++i)
      for(int j = 0; j < eig.cols(); ++j) 
	qpo[j + i*eig.cols()] = eig(i, j);

    return qpo;
  }

  double* StructSVM::eigToQPO(const Eigen::VectorXd& eig)
  {
    double* qpo = new double[eig.rows()];
    for(int i = 0; i < eig.rows(); ++i)
      qpo[i] = eig(i);

    return qpo;
  }
  
  double StructSVM::qpOASES(const Eigen::MatrixXd& H,
			    const Eigen::VectorXd& g,
			    const Eigen::MatrixXd& A,
			    const Eigen::VectorXd& lb,
			    const Eigen::VectorXd& lbA,
			    Eigen::VectorXd* x,
			    bool* feas) const
  {

    // -- Put matrices into the right format.
    double* _H = eigToQPO(H);
    double* _g = eigToQPO(g);
    double* _A = eigToQPO(A);
    double* _lb = eigToQPO(lb);
    double* _lbA = eigToQPO(lbA);
        
    // -- Run the solver.
    QProblem solver(x->rows(), A.rows(), HST_SEMIDEF);
    Options opts;
    opts.setToReliable();
    solver.setOptions(opts);
    solver.setPrintLevel(PL_MEDIUM);
    int nWSR = 100000;
    returnValue rv = solver.init(_H, _g, _A, _lb, NULL, _lbA, NULL, nWSR, NULL);

    // -- Handle errors.
    // Leave x unchanged if infeasible.
    if(rv == RET_INIT_FAILED_INFEASIBILITY || rv == RET_QP_INFEASIBLE) {
      ROS_WARN_STREAM("Infeasible!");
      *feas = false;
      return 0.5 * x->transpose() * H * (*x) + x->dot(g);
    }
    if(rv != SUCCESSFUL_RETURN) {
      ROS_WARN_STREAM("Bad solve!");
      ROS_WARN_STREAM("Return value: " << rv);
      *feas = false;
      return 0.5 * x->transpose() * H * (*x) + x->dot(g);
    }
    ROS_DEBUG_STREAM("qpOASES solved successfully.");
    
    // -- Set the output solution.
    *feas = true;
    double xopt[x->rows()];
    solver.getPrimalSolution(xopt);
    for(int i = 0; i < x->rows(); ++i)
      x->coeffRef(i) = xopt[i];

    // -- Clean up.
    delete[] _H;
    delete[] _g;
    delete[] _A;
    delete[] _lb;
    delete[] _lbA;      
    
    return 0.5 * x->transpose() * H * (*x) + x->dot(g);
  }

  
  double StructSVM::updateWeights(const std::vector<Constraint>& constraints,
				  int num_tr_ex,
				  int num_edge_weights,
				  Eigen::VectorXd* weights,
				  Eigen::VectorXd* slacks,
				  bool* feas) const
  {
    VectorXd x = VectorXd::Zero(weights->rows() + num_tr_ex);
    x.head(weights->rows()) = *weights;

    MatrixXd H = MatrixXd::Zero(x.rows(), x.rows());
    for(int i = 0; i < weights->rows(); ++i)
      H(i, i) = 1.0;

    VectorXd g = VectorXd::Zero(x.rows());
    for(int i = weights->rows(); i < g.rows(); ++i)
      g(i) = c_ / (double)num_tr_ex;

    MatrixXd A = MatrixXd::Zero(constraints.size(), x.rows());
    VectorXd lbA = VectorXd::Zero(constraints.size());
    for(size_t i = 0; i < constraints.size(); ++i) { 
      A.row(i).head(weights->rows()) = constraints[i].dpsi_;
      A(i, weights->rows() + constraints[i].tr_ex_id_) = 1.0;
      lbA(i) = constraints[i].loss_;
    }

    // -- Apply non-negativity constraint to edge weights and slacks.
    VectorXd lb = VectorXd::Ones(x.rows()) * -numeric_limits<double>::max();
    for(int i = 0; i < num_edge_weights; ++i)
      lb(i) = 0;
    for(int i = weights->rows(); i < x.rows(); ++i)
      lb(i) = 0;
    
    // // Everything is non-negative.
    //VectorXd lb = VectorXd::Zero(x.rows());
    
    if(PRINT_MATRICES) { 
      cout << "H" << endl;
      cout << H << endl;
      cout << "g" << endl;
      cout << g.transpose() << endl;
      cout << "A" << endl;
      cout << A << endl;
      cout << "lbA" << endl;
      cout << lbA.transpose() << endl;
      cout << "lb" << endl;
      cout << lb << endl;
    }

    double val = qpOASES(H, g, A, lb, lbA, &x, feas);

    ROS_DEBUG_STREAM("Slack variables: ");
    ROS_DEBUG_STREAM(x.tail(num_tr_ex).transpose());

    if(slacks)
      *slacks = x.tail(num_tr_ex);
    *weights = x.head(weights->rows());

    for(int i = 0; i < num_edge_weights; ++i)
      if(weights->coeffRef(i) < 0)
	weights->coeffRef(i) = 0;
    weights->normalize();
    return val;
  }
  
  double StructSVM::loss(cv::Mat1b label, cv::Mat1b pred, bool hamming) const
  {
    ROS_ASSERT(label.rows == pred.rows);
    ROS_ASSERT(label.cols == pred.cols);

    if(hamming) {
      double loss = 0;
      for(int y = 0; y < label.rows; ++y) {
	for(int x = 0; x < label.cols; ++x) {
	  // Ignore pixels without depth.
	  if(label(y, x) == 255 && pred(y, x) == 0)
	    ++loss;
	  else if(label(y, x) == 0 && pred(y, x) == 255)
	    ++loss;
	}
      }
      return loss;
      //return 1000.0 * loss / (double)(label.rows * label.cols);
    }
    else { 
      bool equal = true;
      for(int y = 0; y < label.rows && equal; ++y) {
	for(int x = 0; x < label.cols && equal; ++x) {
	  // Ignore pixels without depth.
	  if((label(y, x) == 255 && pred(y, x) == 0) ||
	     (label(y, x) == 0 && pred(y, x) == 255))
	    equal = false;
	}
      }
      if(!equal)
	return 1.0;
      else
	return 0.0;
    }

    ROS_ASSERT(0);
    return 0;
  }
  
} // namespace dst
