#ifndef STRINGBUFFER_HPP
#define STRINGBUFFER_HPP
#include <string>
#include <assert.h>
class StringBuffer {
 public:
  template <typename... Args>
  static std::string Format(const char* format, Args... args) {
    int length = std::snprintf(nullptr, 0, format, args...);
    assert(length >= 0);
    char* buf = new char[length + 1];
    std::snprintf(buf, length + 1, format, args...);
    std::string str(buf);
    delete[] buf;
    return str;
  }
};

#endif  // STRINGBUFFER_HPP
