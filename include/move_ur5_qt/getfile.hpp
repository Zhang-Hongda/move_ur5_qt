#if !defined(GETFILE_H)
#define GETFILE_H
#include <dirent.h>
#include <iostream>
#include <regex>
#include <string>

struct file_names {
  std::vector<std::string> all_files;
  std::vector<std::string> ext_files;
};
struct file_names get_files(std::string path, std::string suffix);

#endif  // GETFILE_H
