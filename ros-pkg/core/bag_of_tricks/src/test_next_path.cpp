#include <bag_of_tricks/next_path.h>
#include <bag_of_tricks/bag_of_tricks.h>
#include <gtest/gtest.h>
#include <boost/filesystem.hpp>
#include <ros/assert.h>

using namespace std;
namespace bfs = boost::filesystem;

TEST(nextPath, nextPath)
{
  string dir = "aoeu";
  string prefix = "prefix-";
  string suffix = "-suffix.txt";
  int padding = 4;
  int retval;
  bfs::remove_all(dir);
  bfs::create_directory(dir);

  for(int i = 0; i < 42; ++i) { 
    string path = nextPath(dir, prefix, suffix, padding);
    cout << "Creating path " << path << endl;
    retval = system(("touch " + path).c_str());
    ROS_ASSERT(retval == 0);
  }

  retval = system(("rm " + dir + "/" + prefix + "0033" + suffix).c_str());
  ROS_ASSERT(retval == 0);

  for(int i = 0; i < 5; ++i) { 
    string path = nextPath(dir, prefix, suffix, padding);
    cout << "Creating path " << path << endl;
    retval = system(("touch " + path).c_str());
    ROS_ASSERT(retval == 0);
  }

  EXPECT_TRUE(bfs::exists(dir + "/" + prefix + "0046" + suffix));
  bfs::remove_all(dir);
}

// TODO: This should move to a different test file, or test_next_path.cpp should
// get a more generic name.
TEST(recursiveFind, recursiveFind)
{
  // Directory structure.
  bfs::create_directory("rftest0");
  bfs::create_directory("rftest0/dir0");
  bfs::create_directory("rftest0/dir1");
  bfs::create_directory("rftest0/dir1/dir0");

  // Target files.
  int rv;
  rv = system("touch rftest0/foo.txt");
  rv = system("touch rftest0/dir0/foo.txt");
  rv = system("touch rftest0/dir0/bar.txt");
  rv = system("touch rftest0/dir1/dir0/foo.txt");
  rv = system("touch rftest0/dir1/dir0/bar.txt");

  // Distractor files.
  rv = system("touch rftest0/foo.bin");
  rv = system("touch rftest0/dir0/bar.bin");
  rv = system("touch rftest0/dir1/bar.asoet");
  rv = system("touch rftest0/dir1/dir0/aosnetuh.bin");
  --rv;  // Advanced g++ warning suppression.

  vector<string> res1 = recursiveFind("rftest0", "*.txt");
  cout << "All *.txt files: " << endl;
  copy(res1.begin(), res1.end(), ostream_iterator<string>(cout, "\n"));
  EXPECT_EQ(5, res1.size());

  vector<string> res2 = recursiveFind("rftest0/dir0", "*.txt");
  cout << "*.txt files in /rftest0/dir0: " << endl;
  copy(res2.begin(), res2.end(), ostream_iterator<string>(cout, "\n"));
  EXPECT_EQ(2, res2.size());

  bfs::remove_all("rftest0");
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
