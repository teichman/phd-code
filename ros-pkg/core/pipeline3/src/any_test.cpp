#include <gtest/gtest.h>
#include <pipeline/pipeline.h>
#include <pipeline/common_nodes.h>
#include <pipeline/compute_node_factory.h>
#include <string>

using boost::any_cast;
using boost::any;
using namespace pipeline;
using namespace std;

TEST(Params, Any)
{
  Params p;
  p.set<int>("foo", 13);
  cout << p.get<int>("foo") << endl;
  EXPECT_TRUE(p.get<int>("foo") == 13);
  //cout << p.get<int>("bar") << endl;
}

TEST(Params, Serialize)
{
  Params p;
  p.set<string>("foo", "payload");
  p.set<int>("bar", 13);
  p.set<double>("baz", 42.42);
  //p.set<float>("baz2", 42.42);
  cout << p << endl;
  p.save("paramtest.txt");
  Params p2;
  p2.load("paramtest.txt");
  cout << p2 << endl;
}

TEST(PlaceholderNode, PushAndPull)
{
  EntryPoint<int>* ep = new EntryPoint<int>("IntEntryPoint");
  ep->setData(42);
  
  Params p;
  p.set<int>("foo", 5);
  PlaceholderNode* ph = new PlaceholderNode("PlaceHolder", p);
  ph->registerInput("incoming", ep->getOutlet("outlet"));

  Params p2;
  p2.set<int>("foo", 5);
  PlaceholderNode* ph2 = new PlaceholderNode("PlaceHolder2", p2);
  ph2->registerInput("incoming", ph->getOutlet("outgoing"));
    
  Pipeline pl(1);
  pl.addComponent(ph);
  pl.compute();

  pl.flush();
  ep->setData(37);
  pl.compute();
}

TEST(ComputeNodeFactory, Serialize)
{
  ComputeNodeFactory cnf;

  EntryPoint<int>* ep = new EntryPoint<int>("IntEntryPoint");
  Params p;
  p.set<int>("foo", 5);
  PlaceholderNode* ph = new PlaceholderNode("PlaceHolder", p);
  ph->registerInput("incoming", ep->getOutlet("outlet"));

  Params p2;
  p2.set<int>("foo", 5);
  PlaceholderNode* ph2 = new PlaceholderNode("PlaceHolder2", p2);
  ph2->registerInput("incoming", ep->getOutlet("outlet"));

  Params p3;
  p3.set<int>("foo", 5);
  PlaceholderNode* ph3 = new PlaceholderNode("PlaceHolder3", p3);
  ph3->registerInput("incoming", ph2->getOutlet("outgoing"));
  
  Pipeline pl(1);
  pl.addComponent(ph);
  cout << "====================" << endl;
  cout << "Initial pipeline spec." << endl;
  pl.serialize(cout);

  pl.save("plspec.txt");

  Pipeline pl2(1);
  pl2.load("plspec.txt");
  cout << "====================" << endl;
  cout << "Loaded pipeline spec: " << endl;
  pl2.serialize(cout);
  
  EntryPoint<int>* ep2 = pl2.getNode< EntryPoint<int> >();
  ep2->setData(13);
  cout << "====================" << endl;
  cout << "Deserialized.  Computing." << endl;
  pl2.compute();

  pl2.writeGraphviz("graphvis");
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
