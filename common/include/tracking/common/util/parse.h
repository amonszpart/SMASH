//
// Created by bontius on 04/04/16.
//

#ifndef TRACKVIDEO_COMMON_PARSE_H
#define TRACKVIDEO_COMMON_PARSE_H

#include <vector>

namespace console {
  int  find_argument (int argc, const char** argv, const char* argument_name);
  bool find_switch (int argc, const char** argv, const char* argument_name);
  template <typename T>
  bool parse_arg(int argc, const char **argv, char const *flag, T &value);
  template <typename T>
  int  parse_x_arguments(int argc, char **argv, const char *str, std::vector<T> &v);
} //...ns console

#endif //TRACKVIDEO_COMMON_PARSE_H
