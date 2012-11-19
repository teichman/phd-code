#include <gtest/gtest.h>
#include <asp/asp.h>

using namespace std;
using namespace Eigen;

TEST(ASP, ASPWeights)
{
  ASPWeights weights;
  {
    NameMapping nmap;
    nmap.addName("BilateralNP");
    nmap.addName("DistanceNP");
    nmap.addName("LabelFlowNP");
    NameMapping emap;
    emap.addName("DistanceEP");
    emap.addName("SurfaceNormalEP");
    weights.applyNameMapping("nmap", nmap);
    weights.applyNameMapping("emap", emap);
  }

  cout << weights.status("  ") << endl;

  for(int i = 0; i < weights.nweights_.rows(); ++i) 
    weights.nweights_(i) = i+1.01234;
  for(int i = 0; i < weights.eweights_.rows(); ++i) 
    weights.eweights_(i) = i+1.01;
  cout << weights.status("  ") << endl;

  {
    NameMapping nmap;
    nmap.addName("LabelFlowNP");
    nmap.addName("DistanceNP");
    nmap.addName("BilateralNP");  
    NameMapping emap;
    emap.addName("SurfaceNormalEP");
    emap.addName("DistanceEP");
    weights.applyNameMapping("nmap", nmap);
    weights.applyNameMapping("emap", emap);
  }
  cout << weights.status("  ") << endl;
  EXPECT_TRUE(weights.nweights_(weights.nameMapping("nmap").toId("BilateralNP")) == 1.01234);
  EXPECT_TRUE(weights.eweights_(weights.nameMapping("emap").toId("SurfaceNormalEP")) == 2.01);

  string filename = "asp_weights_test";
  weights.save(filename);

  ASPWeights weights2;
  weights2.load(filename);
  cout << weights2.status("  ") << endl;
  EXPECT_TRUE(weights2.nweights_(weights2.nameMapping("nmap").toId("BilateralNP")) == 1.01234);
  EXPECT_TRUE(weights2.eweights_(weights2.nameMapping("emap").toId("SurfaceNormalEP")) == 2.01);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
