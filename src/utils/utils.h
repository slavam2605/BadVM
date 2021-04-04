#ifndef BADJVM_UTILS_H
#define BADJVM_UTILS_H

#include <cstring>
#include <stdexcept>
#include <sstream>

#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

#define assert(c) \
    if (!(c)) { \
        std::stringstream ss; \
        ss << "Assertion failed in " << __FILENAME__ \
           << " (function '" << __func__ << "', line " << __LINE__ << "): " \
           << #c << std::endl; \
        throw std::runtime_error(ss.str()); \
    }

#define default_fail default: assert(false)

#endif //BADJVM_UTILS_H
