//
// Created by bontius on 26/02/16.
//

#ifndef TRACKVIDEO_COMMON_OS_H
#define TRACKVIDEO_COMMON_OS_H


#include <sys/stat.h>
#include <dirent.h>
#include <string>
#include <iostream>
#include <vector>

namespace io {
static constexpr char kFileSep = '/';
inline void my_mkdir( std::string path,
                      unsigned mode = S_IRUSR | S_IWUSR | S_IXUSR | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH ) {
    mkdir(path.c_str(),mode);
}

/** \brief Retrieves the filename stem from a path separated by \p pathSep and having the extension after '.'. */
inline std::string getStem(std::string const& path, const char pathSep = kFileSep) {
    std::size_t start = path.rfind(pathSep)+1;
    if (start == std::string::npos)
        start = 0;
    std::size_t stop = path.rfind('.');
    if (stop == std::string::npos)
        return path.substr(start);
    else
        return path.substr(start,stop - start);
} //...getStem()

inline std::string getExtension(std::string const& path) {
    return path.substr(path.rfind('.')+1);
} //...getExtension()

inline std::vector<std::string> walk(std::string const& dir, std::string const& extension = "") {
    std::vector<std::string> files;

    DIR *dp;
    struct dirent *dirp;
    if ((dp = opendir(dir.c_str())) == NULL) {
        std::cerr << "Error(" << errno << ") opening " << dir << std::endl;
        return files;
    }

    while ((dirp = readdir(dp)) != NULL) {
        auto const n = std::string(dirp->d_name);
        if (extension.empty() || (io::getExtension(n) == extension) )
            files.push_back(n);
    }
    closedir(dp);
    return files;
} //...walk()

inline bool exists (const std::string& name) {
    struct stat buffer;
    return (stat (name.c_str(), &buffer) == 0);
}

inline void removeFile(std::string const& name)
{ std::remove(name.c_str()); }

} //...ns io

#endif //TRACKVIDEO_COMMON_OS_H
