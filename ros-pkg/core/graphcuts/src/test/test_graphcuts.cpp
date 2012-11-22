#include <gtest/gtest.h>
#include <graphcuts/maxflow_inference.h>
#include <graphcuts/structural_svm.h>

using namespace Eigen;
using namespace std;
namespace gc = graphcuts;

TEST(Model, Serialization)
{
  VectorXd epot_weights(3);
  epot_weights << 0, M_PI, 42.13;
  NameMapping epot_names;
  epot_names.addName("edge_descriptor0");
  epot_names.addName("edge_descriptor1");
  epot_names.addName("edge_descriptor2");
  VectorXd npot_weights(2);
  npot_weights << M_PI, 42.13;
  NameMapping npot_names;
  npot_names.addName("node_descriptor0");
  npot_names.addName("node_descriptor1");

  gc::Model model(epot_weights, npot_weights, epot_names, npot_names);
  model.save("test_model");
  cout << model << endl;
  
  gc::Model model2;
  model2.load("test_model");
  cout << model2 << endl;
  
  EXPECT_TRUE(model.npot_names_ == model2.npot_names_);
  EXPECT_TRUE(model.epot_names_ == model2.epot_names_);
  EXPECT_TRUE(model.concatenate().rows() == model2.concatenate().rows());
  for(int i = 0; i < model2.concatenate().rows(); ++i)
    EXPECT_FLOAT_EQ(model.concatenate()(i), model2.concatenate()(i));

  VectorXd psi(5);
  psi << 0, 1, 2, 3, 4;
  EXPECT_FLOAT_EQ(model.concatenate().dot(psi), model.score(psi));
}

TEST(StructuralSVM, SanityCheck)
{
  double c = 10;
  double precision = 1e-4;
  int num_threads = 1;
  int debug_level = 0;
  gc::StructuralSVM ssvm(c, precision, num_threads, debug_level);

  // -- Generate data.
  gc::PotentialsCache::Ptr cache(new gc::PotentialsCache);

  gc::VecXiPtr label(new gc::VecXi(10));
  *label << 0, 0, 0, 0, 0, 1, 1, 1, 1, 1;

  VectorXd good_sink(10);
  good_sink << 0, 0, 0, 0, 1, 0, 0, 0, 0, 0;
  VectorXd good_source(10);
  good_source << 0, 0, 0, 0, 0, 1, 0, 0, 0, 0;
  cache->npot_names_.addName("GoodDescriptor");
  cache->sink_.push_back(good_sink);
  cache->source_.push_back(good_source);
  cache->npot_names_.addName("ReversedDescriptor");
  cache->sink_.push_back(-good_sink);
  cache->source_.push_back(-good_source);

  cache->epot_names_.addName("GoodEdge");
  gc::DynamicSparseMat good_edge(10, 10);
  for(int i = 1; i < good_edge.rows(); ++i)
    good_edge.coeffRef(i-1, i) = 1.0;
  cache->edge_.push_back(gc::SparseMat(good_edge));

  cache->epot_names_.addName("BadEdge");
  gc::DynamicSparseMat bad_edge(10, 10);
  bad_edge.coeffRef(4, 5) = 100;
  cache->edge_.push_back(bad_edge);

  cache->symmetrizeEdges();
    
  // -- Train model.
  vector<gc::PotentialsCache::Ptr> caches;
  vector<gc::VecXiPtr> labels;
  caches.push_back(cache);
  labels.push_back(label);
  gc::Model model = ssvm.train(caches, labels);
  cout << model << endl;

  // -- Run inference on data and make sure it's reasonable.
  gc::MaxflowInference mfi(model);
  VectorXi seg;
  mfi.segment(cache, &seg);
  EXPECT_TRUE(cache->getNumNodes() == seg.rows());
  cout << label->transpose() << endl;
  cout << seg.transpose() << endl;

  for(int i = 0; i < seg.rows(); ++i)
    EXPECT_TRUE(seg(i) == label->coeffRef(i));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
