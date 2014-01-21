#include <gtest/gtest.h>
#include <online_learning/synthetic_data_generator.h>

using namespace std;
using namespace Eigen;

class ExampleCustomSerializer : public CustomSerializer
{
public:

  std::string name() const { return "ExampleCustomSerializer"; }
  
  void serialize(const boost::any& raw, std::ostream& out) const
  {
    int val = boost::any_cast<int>(raw);
    out.write((char*)&val, sizeof(val));
  }
  
  void deserialize(std::istream& in, boost::any* raw) const
  {
    int val;
    in.read((char*)&val, sizeof(val));
    *raw = val;
  }
};

TEST(CustomSerializer, CustomSerializer)
{
  // -- Set the custom serializer so that any serialization of Instances
  //    will correctly handle the custom data.
  Instance::custom_serializer_ = CustomSerializer::Ptr(new ExampleCustomSerializer);

  // -- Generate a sample dataset.
  int num_descriptors = 3;
  SyntheticDataGenerator sdg(getDefaultDimensionality(num_descriptors), 10, 1, 0.2, defaultClassMap(), getStubDescriptorMap(num_descriptors));
  Dataset::Ptr dataset = sdg.sampleDataset(42);

  // -- Add some custom data.
  int idx = 0;
  for(size_t i = 0; i < dataset->size(); ++i, ++idx)
    (*dataset)[i].raw_ = idx;

  // -- Serialize using custom serializer set above.
  dataset->save("augmented_dataset");

  // -- Load and check that the custom data is the same.
  Dataset dataset2;
  dataset2.load("augmented_dataset");
  idx = 0;
  cout << endl;
  for(size_t i = 0; i < dataset->size(); ++i, ++idx) {
    EXPECT_EQ(boost::any_cast<int>((*dataset)[i].raw_), boost::any_cast<int>(dataset2[i].raw_));
    EXPECT_EQ(boost::any_cast<int>(dataset2[i].raw_), idx);
    cout << boost::any_cast<int>(dataset2[i].raw_) << " ";
  }
  cout << endl;

  // -- Remove the custom serializer and make sure
  //    we can load the dataset (albeit without the custom data).
  Instance::custom_serializer_ = CustomSerializer::Ptr(new EmptyCustomSerializer);
  Dataset ncd;
  ncd.load("augmented_dataset");
  EXPECT_EQ(ncd.size(), dataset2.size());
  EXPECT_TRUE(ncd == dataset2);  // op== ignores custom data.

  // -- Test passthrough serializer.
  Instance::custom_serializer_ = CustomSerializer::Ptr(new PassthroughCustomSerializer);
  Dataset ptd;  // passthrough td
  ptd.load("augmented_dataset");
  EXPECT_EQ(ptd.size(), dataset2.size());
  EXPECT_TRUE(ptd == dataset2);  // op== ignores custom data.
  ptd.save("augmented_dataset-passthrough");

  Instance::custom_serializer_ = CustomSerializer::Ptr(new ExampleCustomSerializer);
  Dataset aptd;  // after passthrough td
  aptd.load("augmented_dataset-passthrough");
  EXPECT_EQ(aptd.size(), dataset2.size());
  EXPECT_TRUE(aptd == dataset2);  // op== ignores custom data.
}

TEST(CustomSerializer, ReadOnlyEmptyCustomSerializerDeathTest)
{
  // -- Set the custom serializer so that any serialization of Instances
  //    will correctly handle the custom data.
  Instance::custom_serializer_ = CustomSerializer::Ptr(new ExampleCustomSerializer);

  // -- Generate a sample dataset.
  int num_descriptors = 3;
  SyntheticDataGenerator sdg(getDefaultDimensionality(num_descriptors), 10, 1, 0.2, defaultClassMap(), getStubDescriptorMap(num_descriptors));
  Dataset::Ptr dataset = sdg.sampleDataset(42);

  // -- Add some custom data.
  int idx = 0;
  for(size_t i = 0; i < dataset->size(); ++i, ++idx)
    (*dataset)[i].raw_ = idx;

  // -- Serialize using custom serializer set above.
  dataset->save("augmented_dataset");

  // -- Set a CustomSerializer that will not allow writing.
  Instance::custom_serializer_ = CustomSerializer::Ptr(new ReadOnlyEmptyCustomSerializer);

  Dataset d2;
  d2.load("augmented_dataset");  // should work.

  // -- Try to clobber the raw data.  This should die.
  //    ...  Unfortunately gtest can't do death tests on linux.  WTF people.
  //    http://stackoverflow.com/questions/14062628/how-to-make-google-test-detect-the-number-of-threads-on-linux

  //d2.save("augmented_dataset");
  //EXPECT_DEATH({ d2.save("augmented_dataset"); }, "");
}


int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
