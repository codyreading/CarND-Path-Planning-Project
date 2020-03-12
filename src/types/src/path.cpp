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

int Path::size()
{
    return m_x.size();
}

void Path::add(double x, double y,
               double s, double s_vel, double s_acc, double s_jerk,
               double d, double d_vel, double d_acc, double d_jerk,
               double yaw)
{
    m_x.push_back(x);
    m_y.push_back(y);
    m_s.push_back(s);
    m_d.push_back(d);
    m_s_vel.push_back(s_vel);
    m_s_acc.push_back(s_acc);
    m_s_jerk.push_back(s_jerk);
    m_d_vel.push_back(d_vel);
    m_d_acc.push_back(d_acc);
    m_d_jerk.push_back(s_jerk);
    m_yaw.push_back(yaw);
}

void Path::removeFirstPoints(int numPoints)
{
    m_x.erase(m_x.begin(), m_x.begin() + numPoints);
    m_y.erase(m_y.begin(), m_y.begin() + numPoints);
    m_s.erase(m_s.begin(), m_s.begin() + numPoints);
    m_d.erase(m_d.begin(), m_d.begin() + numPoints);
    m_s_vel.erase(m_s_vel.begin(), m_s_vel.begin() + numPoints);
    m_d_vel.erase(m_d_vel.begin(), m_d_vel.begin() + numPoints);
    m_s_acc.erase(m_s_acc.begin(), m_s_acc.begin() + numPoints);
    m_d_acc.erase(m_d_acc.begin(), m_d_acc.begin() + numPoints);
    m_s_jerk.erase(m_s_jerk.begin(), m_s_jerk.begin() + numPoints);
    m_d_jerk.erase(m_d_jerk.begin(), m_d_jerk.begin() + numPoints);
    m_d_jerk.erase(m_yaw.begin(), m_yaw.begin() + numPoints);
}

Path Path::clone(int up_to_index)
{
    Path copy = Path();
    for (int i = 0; i < size() && i < up_to_index; ++i)
    {
        copy.add(m_x[i], m_y[i],
                 m_s[i], m_s_vel[i], m_s_acc[i], m_s_jerk[i],
                 m_d[i], m_d_vel[i], m_d_acc[i], m_d_jerk[i],
                 m_yaw[i]);
    }
    return copy;
}