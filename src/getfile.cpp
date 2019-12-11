#include "../include/move_ur5_qt/getfile.h"

struct file_names get_files(std::string path, std::string suffix) {
  struct dirent *dirp;
  file_names fn;
  DIR *dp;

  dp = opendir(path.c_str());
  std::regex reg_obj(".*" + suffix);
  while ((dirp = readdir(dp)) != NULL) {
    if (dirp->d_type == 8)  // 4 means catalog; 8 means file; 0 means unknown
    {
      fn.all_files.push_back(dirp->d_name);
      if (std::regex_match(dirp->d_name, reg_obj)) {
        std::string all_path = path + "/" + dirp->d_name;
        fn.ext_files.push_back(all_path);
      }
    }
  }
  std::sort(fn.all_files.begin(), fn.all_files.end());
  std::sort(fn.ext_files.begin(), fn.ext_files.end());
  closedir(dp);
  return fn;
}
