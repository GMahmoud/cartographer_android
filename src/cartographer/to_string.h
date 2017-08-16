#ifndef CARTOGRAPHER_CUSTOM_TOSTR
#define CARTOGRAPHER_CUSTOM_TOSTR
#include <sstream>

template <typename T>
std::string to_string(const T& value) {
    std::stringstream ss;
    ss << value;
    return ss.str();
}
#endif //CARTOGRAPHER_CUSTOM_TOSTR
