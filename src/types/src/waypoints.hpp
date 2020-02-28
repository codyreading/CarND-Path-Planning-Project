#ifndef WAYPOINTS_HPP
#define WAYPOINTS_HPP

struct Waypoints
{
    std::vector<double> x; // X positions in map coordinates
    std::vector<double> y; // Y positions in map coordinates
    std::vector<double> s; // Distances to get to that waypoint from the last waypoint
    std::vector<double> dx; // X components of unit normal vector pointing outward of the highway loop.
    std::vector<double> dy; // Y components of unit normal vector pointing outward of the highway loop.
};

#endif