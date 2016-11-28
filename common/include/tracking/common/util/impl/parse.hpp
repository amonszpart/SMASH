#ifndef SOUP_PARSE_HPP
#define SOUP_PARSE_HPP
//
// Created by bontius on 12/06/15.
//

#include <cstdlib>
#include <string>           // std::string
#include <cstring>          // strcmp
#include <vector>
#include <sstream>

namespace console {
/*! \brief Finds a flag and its value in the argument list.
     *  \note Borrowed from PCL.
     */
inline int find_argument (int argc, const char** argv, const char* argument_name)
{
    for (int i = 1; i < argc; ++i)
    {
        // Search for the string
        if (strcmp (argv[i], argument_name) == 0)
        {
            return (i);
        }
    }
    return (-1);
}

/*! \brief Finds a flag in the argument list.
     *  \note Borrowed from PCL.
     */
inline bool find_switch (int argc, const char** argv, const char* argument_name)
{
    return (find_argument (argc, argv, argument_name) != -1);
}

template<typename T>
bool parse_var(const char *str, T &val);

template<>
inline bool parse_var(const char *str, float &val) {
    val = std::stof(str);
    return true;
}

template<>
inline bool parse_var(const char *str, double &val) {
    val = std::stof(str);
    return true;
}

template<>
inline bool parse_var(const char *str, int &val) {
    val = std::stoi(str);
    return true;
}

template<>
inline bool parse_var(const char *str, std::string &val) {
    val = std::string(str);
    return true;
}

template<>
inline bool parse_var(const char *str, unsigned &val) {
    val = std::stoi( str );
    return true;
}

template<>
inline bool parse_var(const char *str, size_t &val) {
    val = std::stol( str );
    return true;
}

template<>
inline bool parse_var(const char *str, int64_t &val) {
    val = std::stol(str);
    return true;
}

template<typename T>
inline bool parse_arg(int argc, const char **argv, char const *flag, T &value) {
    int i = 1;
    while (i < argc) {
        if (!strcmp(flag, argv[i])) {
            if (i + 1 < argc) {
                return parse_var(argv[i + 1], value);
                //value = atof( argv[i+1] );
                //return true;
            }
        }
        ++i;
    }
    return false;
}

template<typename T>
inline int
parse_x_arguments(int argc, const char **argv, const char *str, std::vector<T> &v)
{
    for (int i = 1; i < argc; ++i) {
        // Search for the string
        if ((strcmp(argv[i], str) == 0) && (++i < argc)) {
            // look for ',' as a separator
            std::vector<std::string> values;

            std::stringstream ss(argv[i]);
            std::string s;
            while (getline(ss, s, ',')) values.push_back(s);

            v.resize(values.size());
            for (size_t j = 0; j < v.size(); ++j) {
                //v[j] = atoi (values.at (j).c_str ());
                parse_var( values.at(j).c_str(), v[j] );
            }

            return (i - 1);
        }
    }
    return (-1);
}
} //...ns console


#endif // SOUP_PARSE_HPP
