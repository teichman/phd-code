#ifndef NEXT_PATH_H
#define NEXT_PATH_H

#include <string>

//! Saves to dir/[prefix][num][suffix], filling in [num] appropriately.
//! TODO: std::string path = nextPath("path/to/dir/prefix-####-suffix.aoeu") would be a nicer API.
std::string nextPath(std::string dir, std::string prefix, std::string suffix, int padding);

#endif // NEXT_PATH_H

