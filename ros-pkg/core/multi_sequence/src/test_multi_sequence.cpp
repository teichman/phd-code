#include <multi_sequence/multi_sequence.h>
#include <boost/filesystem.hpp>
#include <gtest/gtest.h>

using namespace multi_sequence;
using namespace std;

TEST( TestMultiSequence, MinMax ) {
  vector<int> vals;
  vals.push_back(3);
  vals.push_back(2);
  vals.push_back(1);
  int minv, maxv;
  size_t mini, maxi;
  minMax( vals, minv, mini, maxv, maxi );
  EXPECT_EQ( minv, 1 );
  EXPECT_EQ( maxv, 3 );
  EXPECT_EQ( mini, 2 );
  EXPECT_EQ( maxi, 0 );
}


TEST( TestMultiSequence, SingleSequence ) {
  rgbd::Sequence::Ptr seq( new rgbd::Sequence );
  seq->load("test/seq01");
  MultiSequence ms(0.01);
  ms.addSequence(seq);
  EXPECT_EQ( seq->size(), ms.size() );
  for(size_t i = 0; i < ms.size(); i++)
  {
    vector<cv::Mat3b> imgs;
    ms.getImages(i, imgs);
    vector<cv::Mat1b> imgs_c,seq_imgs_c;
    cv::split( imgs[0], imgs_c );
    cv::split( seq->imgs_[i], seq_imgs_c );
    for(size_t i = 0; i < imgs_c.size(); i++){
      EXPECT_EQ( cv::countNonZero( imgs_c[i] != seq_imgs_c[i]), 0 );
    }
  }
  //Adding the same sequence shouldn't hurt anything
  rgbd::Sequence::Ptr seq2( new rgbd::Sequence(*seq) );
  seq2->imgs_.erase( seq2->imgs_.begin(),seq2->imgs_.begin()+2 ); //Erase first 2
  seq2->pcds_.erase( seq2->pcds_.begin(),seq2->pcds_.begin()+2 ); //Erase first 2
  ms.addSequence(seq2);
  EXPECT_EQ( seq2->size(), ms.size() );
  EXPECT_EQ( seq->size(), ms.size()+2 );
  for(size_t i = 0; i < ms.size(); i++)
  {
    vector<cv::Mat3b> imgs;
    ms.getImages(i, imgs);
    vector<cv::Mat1b> imgs_c,seq_imgs_c;
    cv::split( imgs[0], imgs_c );
    cv::split( seq->imgs_[i+2], seq_imgs_c );
    for(size_t i = 0; i < imgs_c.size(); i++){
      ASSERT_EQ( cv::countNonZero( imgs_c[i] != seq_imgs_c[i]), 0 );
    }
  }
  ms.save("test/multi");
  MultiSequence ms2;
  ms2.load("test/multi");
  for(size_t i = 0; i < ms2.size(); i++)
  {
    vector<cv::Mat3b> imgs;
    ms2.getImages(i, imgs);
    vector<cv::Mat1b> imgs_c,seq_imgs_c;
    cv::split( imgs[0], imgs_c );
    cv::split( seq->imgs_[i+2], seq_imgs_c );
    for(size_t i = 0; i < imgs_c.size(); i++){
      ASSERT_EQ( cv::countNonZero( imgs_c[i] != seq_imgs_c[i]), 0 );
    }
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

