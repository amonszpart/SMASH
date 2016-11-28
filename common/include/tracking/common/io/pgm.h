#ifndef TV_PGM_H
#define TV_PGM_H

#include <string>

namespace cv { template <class T> class Mat_; }
namespace tracking
{
    bool readPgm( const std::string& path, cv::Mat_<float>& depth, const float scale );

    template <typename F>
    std::string one_line( F *fp )
    {
        int c;
        int pos = 0;
        char *out = NULL;
        for (c = fgetc(fp); !(c == '\n' || c == EOF); c = fgetc(fp))
        {
            out = static_cast<char*>( realloc(out, pos + 1) );
            out[pos++] = c;
        }
        if (out) {
            out = static_cast<char*>( realloc(out, pos + 1) );
            out[pos] = '\0';
        }
        return out;
    }
}

#endif // TV_PGM_H

