#ifndef STATE_HPP
#define STATE_HPP

/* Vehicle state*/
struct State
{
    double x; // X position in map coordinates
    double y; // Y position in map coordinates
    double s; // S position in frenet coordinates
    double d; // D position in frenet coordinates
    double yaw; // Yaw angle in map
    double speed; // Speed in MPH
};

#endif