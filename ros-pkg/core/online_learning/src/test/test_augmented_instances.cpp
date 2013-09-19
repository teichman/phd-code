#include <gtest/gtest.h>
#include <online_learning/augmented_instance_experiment.h>

TEST(AugmentedInstance, AugmentedInstance)
{
  Instance<Bare> inst;
  inst.val_ = 13;
  inst.save("unaugmented_instance");
  inst.something();
  Instance<Bare> inst2;
  inst2.load("unaugmented_instance");
  EXPECT_TRUE(inst.val_ == inst2.val_);

  Instance<ExtraCustomData> aug;
  aug.val_ = 42;
  aug.raw_.val_ = 13;
  aug.save("augmented_instance");

  Instance<ExtraCustomData> aug2;
  aug2.load("augmented_instance");
  EXPECT_TRUE(aug2.val_ == aug.val_);
  EXPECT_TRUE(aug2.raw_.val_ == aug.raw_.val_);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
