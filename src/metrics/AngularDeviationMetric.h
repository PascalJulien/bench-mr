#pragma once

#include "TrajectoryMetric.h"
#include "PathLengthMetric.h"


class AngularDeviationMetric : public TMetric<AngularDeviationMetric> {
 public:
  static double evaluateMetric(const ompl::geometric::PathGeometric& trajectory,double) {
    if (trajectory.getStateCount() < 2) return 0.0;

    //const auto* firstState = trajectory.getState(0)->as<ob::SE2StateSpace::StateType>();
    //double firstTheta = firstState->getYaw();
   
    const auto* lastState = trajectory.getState(trajectory.getStateCount() - 1)->as<ob::SE2StateSpace::StateType>();
    const auto* ante_lastState = trajectory.getState(trajectory.getStateCount() - 2)->as<ob::SE2StateSpace::StateType>();
    const auto* _ante_ante_lastState = trajectory.getState(trajectory.getStateCount() - 3)->as<ob::SE2StateSpace::StateType>();
    double lastTheta = lastState->getYaw();
    double ante_lastTheta = ante_lastState->getYaw();
    double ante_ante_lastTheta = _ante_ante_lastState->getYaw();
    
    double goalTheta = global::settings.environment->goalTheta();

    double thetaDeviation = fmin(std::abs(lastTheta - goalTheta),2*M_PI-std::abs(lastTheta - goalTheta))+
                            fmin(std::abs(ante_lastTheta - lastTheta),2*M_PI-std::abs(ante_lastTheta - lastTheta))+
                            fmin(std::abs(ante_ante_lastTheta - ante_lastTheta),2*M_PI-std::abs(ante_ante_lastTheta - ante_lastTheta)); //+ std::abs(firstTheta - startTheta);

    double length = PathLengthMetric::evaluateMetric(trajectory, 0.0);
    double length_per_points = length /trajectory.getStateCount();
    thetaDeviation =  thetaDeviation / (3*length_per_points);

    return thetaDeviation;
  }

  static double evaluateMetric(const ompl::control::PathControl& trajectory,double) {
    if (trajectory.getStateCount() < 2) return 0.0;

    //const auto* firstState = trajectory.getState(0)->as<ob::SE2StateSpace::StateType>();
    //double firstTheta = firstState->getYaw();
   
    const auto* lastState = trajectory.getState(trajectory.getStateCount() - 1)->as<ob::SE2StateSpace::StateType>();
    const auto* ante_lastState = trajectory.getState(trajectory.getStateCount() - 2)->as<ob::SE2StateSpace::StateType>();
    const auto* _ante_ante_lastState = trajectory.getState(trajectory.getStateCount() - 3)->as<ob::SE2StateSpace::StateType>();
    double lastTheta = lastState->getYaw();
    double ante_lastTheta = ante_lastState->getYaw();
    double ante_ante_lastTheta = _ante_ante_lastState->getYaw();
    
    double goalTheta = global::settings.environment->goalTheta();
    double thetaDeviation = fmin(std::abs(lastTheta - goalTheta),2*M_PI-std::abs(lastTheta - goalTheta))+
                            fmin(std::abs(ante_lastTheta - lastTheta),2*M_PI-std::abs(ante_lastTheta - lastTheta))+
                            fmin(std::abs(ante_ante_lastTheta - ante_lastTheta),2*M_PI-std::abs(ante_ante_lastTheta - ante_lastTheta)); //+ std::abs(firstTheta - startTheta);

    double length = PathLengthMetric::evaluateMetric(trajectory, 0.0);
    double length_per_points = length / trajectory.getStateCount();
    thetaDeviation =  thetaDeviation / (3*length_per_points);

    return thetaDeviation;
  }

};