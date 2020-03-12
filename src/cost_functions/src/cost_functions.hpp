#ifndef COST_FUNCTIONS_HPP
#define COST_FUNCTIONS_HPP

#include <vector>
#include "vehicle.hpp"
#include "path.hpp"
#include "state_machine.hpp"


typedef function<double (const Vehicle&, const std::vector<Vehicle>&,  Path&, const State&, const double&)> CostFunction;


double speedCostFunction(const Vehicle& ego, const std::vector<Vehicle>& others, Path& path,
                         const State& state, const double& weight);


double centerOfLaneDistCostFunction(const Vehicle& ego, const std::vector<Vehicle>& others, Path& path,
                                    const State& state, const double& weight);


double laneChangeCostFunction(const Vehicle& ego, const std::vector<Vehicle>& others, Path& path,
                              const State& state, const double& weight);


double distanceToClosestCarAheadCostFunction(const Vehicle& ego, const std::vector<Vehicle>& others, Path& path,
    const State& state, const double& weight);


double longitudinalDistanceToClosestAdjacentCarFunction(const Vehicle& ego, const std::vector<Vehicle>& others,  Path& path,
    const State& state, const double& weight);


double distanceToClosestCarAheadFutureLaneCostFunction(const Vehicle& ego, const std::vector<Vehicle>& others,  Path& path,
    const State& state, const double& weight);


double averageLaneSpeedDiffCostFunction(const Vehicle& ego, const std::vector<Vehicle>& others,  Path& path,
                                        const State& state, const double& weight);



double speedDifferenceWithClosestCarAheadCostFunction(const Vehicle& ego, const std::vector<Vehicle>& others,  Path& path,
    const State& state, const double& weight);


double lanesAverageForwardSpeedCarsAhead(const Vehicle& ego, const std::vector<Vehicle>& others,  Path& path,
    const State& state, const double& weight);




double collisionTimeCostFunction(const Vehicle& ego, const std::vector<Vehicle>& others,  Path& path,
                                 const State& state, const double& weight);




double futureDistanceToGoalCostFunction(const Vehicle& ego, const std::vector<Vehicle>& others,  Path& path,
                                        const State& state, const double& weight);


#endif