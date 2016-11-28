//
// Created by bontius on 31/01/16.
//

#ifndef TRACKVIDEO_EXCEPTION_H
#define TRACKVIDEO_EXCEPTION_H

#include <stdexcept>
#include <string>
#include <iostream>

#define DEFINE_EXCEPTION_WITH_BASE(ClassName, BaseError) \
    class ClassName##Exception : public BaseError { \
        public: \
            explicit ClassName##Exception(const char *msg,  \
                                          const char* sourceFile = "", const int lineNumber = -1) \
                : BaseError(msg) \
            {\
                std::cerr << "[" << #ClassName << "]"; \
                if (!std::string(sourceFile).empty()) \
                    std::cerr << "[" << sourceFile; \
                if (lineNumber >= 0) \
                    std::cerr << ":" << lineNumber; \
                std::cerr << "] " << this->what() << std::endl; \
            } \
            \
            explicit ClassName##Exception(std::string msg = std::string(#ClassName) + "Exception", \
                                          const char* sourceFile = "", const int lineNumber = -1) \
                : BaseError(msg.c_str()) \
            {\
                std::cerr << "[" << #ClassName << "]"; \
                if (!std::string(sourceFile).empty()) \
                    std::cerr << "[" << sourceFile; \
                if (lineNumber >= 0) \
                    std::cerr << ":" << lineNumber; \
                std::cerr << "] " << this->what() << std::endl; \
            } \
    }; //...ClassName##Exception
#define DEFINE_EXCEPTION(ClassName) DEFINE_EXCEPTION_WITH_BASE(ClassName, std::runtime_error)

namespace tracking {
DEFINE_EXCEPTION_WITH_BASE(Logic, std::logic_error)
#if 0
/** \brief General exception, that actually dumps its message on construction...*/
class LogicException : public std::logic_error {
public:
    explicit LogicException(const char *msg, const char* sourceFile = "", const int lineNumber = -1)
        : std::logic_error(msg) {
        std::cerr << "[LogicException]";
        if (!std::string(sourceFile).empty())
            std::cerr << "[" << sourceFile << "]";
        if (lineNumber >= 0)
            std::cerr << ":" << lineNumber;
        std::cerr << "] " << this->what() << std::endl;
    }

    explicit LogicException(std::string msg = "LogicException", const char* sourceFile = "", const int lineNumber = -1)
        : std::logic_error(msg.c_str()) {
        std::cerr << "[";
        if (!std::string(sourceFile).empty())
            std::cerr << sourceFile;
        else
            std::cerr << "LogicException";
        if (lineNumber >= 0)
            std::cerr << ":" << lineNumber;
        std::cerr << "] " << this->what() << std::endl;
    }
}; //...LogicException()
#endif
} //...ns tracking

#endif //TRACKVIDEO_EXCEPTION_H
