#include "path.hpp"

Path::Path()
{
}

Path::Path(const std::vector<double>& x, const std::vector<double>& y)
{
    m_x = x;
    m_y = y;
    m_size = x.size();
}

Path::Path(nlohmann::basic_json<> x, nlohmann::basic_json<> y)
{
    for (int i = 0; i < x.size(); ++i)
    {
        m_x.push_back(x[i]);
        m_y.push_back(y[i]);
    }
    m_size = x.size();
}