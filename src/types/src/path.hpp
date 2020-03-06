#ifndef PATH_HPP
#define PATH_HPP

#include <vector>
#include "json.hpp"

class Path
{
public:
    Path();
    Path(const std::vector<double>& x, const std::vector<double>& y);
    Path(nlohmann::basic_json<> x, nlohmann::basic_json<> y);
    std::vector<double> m_x; // List of x coordinates to define a path
    std::vector<double> m_y; // List of y coordinates to define a path
    int m_size; // Number of elements in path
};

#endif