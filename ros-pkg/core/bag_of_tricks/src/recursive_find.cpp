#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <bag_of_tricks/glob.h>
#include <ros/assert.h>

using namespace std;
namespace bfs = boost::filesystem;

std::vector<std::string> recursiveFind(const std::string& dir, const std::string& pattern)
{
  // bfs::rdi won't report its root directory, so we check that one at the start.
  vector<string> all_paths = glob(dir + "/" + pattern);

  // Now search all subdirectories recursively.
  bfs::recursive_directory_iterator it(dir), eod;
  BOOST_FOREACH(const bfs::path& p, make_pair(it, eod)) {
    cout << "Checking: " << p.string() << endl;
    if(bfs::is_directory(p)) {
      cout << "globbing: " << p.string() + "/" + pattern << endl;
      vector<string> paths = glob(p.string() + "/" + pattern);
      all_paths.insert(all_paths.end(), paths.begin(), paths.end());
    }
  }
  
  return all_paths;
}
