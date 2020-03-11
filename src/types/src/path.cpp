#include "path.hpp"

Path::Path()
{
}

Path::Path(const std::vector<double>& x, const std::vector<double>& y)
{
    m_x = x;
    m_y = y;
}

Path::Path(nlohmann::basic_json<> x, nlohmann::basic_json<> y)
{
    for (int i = 0; i < x.size(); ++i)
    {
        m_x.push_back(x[i]);
        m_y.push_back(y[i]);
    }
}

int Path::getSize()
{
    return m_x.size();
}

void Path::removeFirstPoints(int numPoints)
{
    m_x.erase(m_x.begin(), m_x.begin() + numPoints);
    m_y.erase(m_y.begin(), m_y.begin() + numPoints);
}