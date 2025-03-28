#pragma once

#include "TrajectoryMetric.h"
#include "PathLengthMetric.h"



class EuclideanDeviationMetric : public TMetric<EuclideanDeviationMetric> {
 public:
  static double evaluateMetric(const ompl::geometric::PathGeometric& trajectory,double) {
    if (trajectory.getStateCount() < 2) return 0.0;

    const auto path = Point::fromPath(trajectory);

    //const auto& firstPoint = path.front();
    const auto& lastPoint = path.back();
    const auto& beforeLastPoint = path[path.size() - 2];
    const auto& beforebeforeLastPoint = path[path.size() - 3];

    //const auto& start = global::settings.environment->start();
    const auto& goal = global::settings.environment->goal();

    double deviation = computeDeviation(lastPoint, goal);
    deviation += computeDeviation(beforeLastPoint, lastPoint);
    deviation += computeDeviation(beforebeforeLastPoint, beforeLastPoint);
    double length = PathLengthMetric::evaluateMetric(trajectory, 0.0);
    double length_per_points = length / path.size();
    deviation =  deviation / (3*length_per_points);
    return deviation;
  }

  static double evaluateMetric(const ompl::control::PathControl& trajectory,double) {
    if (trajectory.getStateCount() < 2) return 0.0;

    const auto path = Point::fromPath(trajectory);

    //const auto& firstPoint = path.front();
    const auto& lastPoint = path.back();
    const auto& beforeLastPoint = path[path.size() - 2];
    const auto& beforebeforeLastPoint = path[path.size() - 3];

    //const auto& start = global::settings.environment->start();
    const auto& goal = global::settings.environment->goal();

    double deviation = computeDeviation(lastPoint, goal);
    deviation += computeDeviation(beforeLastPoint, lastPoint);
    deviation += computeDeviation(beforebeforeLastPoint, beforeLastPoint);
    double length = PathLengthMetric::evaluateMetric(trajectory, 0.0);
    double length_per_points = length / path.size();
    deviation =  deviation / (3*length_per_points);
    return deviation;
  }

 private:
  static double computeDeviation(const Point& lastPoint, const Point& goal) {
    // double dx_start = firstPoint.x - start.x;
    // double dy_start = firstPoint.y - start.y;
    // double distance_start = std::sqrt(dx_start * dx_start + dy_start * dy_start);

    double dx_goal = lastPoint.x - goal.x;
    double dy_goal = lastPoint.y - goal.y;
    double distance_goal = std::sqrt(dx_goal * dx_goal + dy_goal * dy_goal);

    return distance_goal; //+ distance_start;
  }
};