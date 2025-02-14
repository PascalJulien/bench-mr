#pragma once

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "metrics/PathLengthMetric.h"
#include "metrics/MaxCurvatureMetric.h"
#include "metrics/NormalizedCurvatureMetric.h"
#include "utils/PlannerUtils.hpp"

namespace py = pybind11;

namespace metrics {

// Fonction wrapper pour calculer la longueur du chemin
double compute_path_length(const std::vector<std::vector<double>>& path);

// Fonction wrapper pour calculer la courbure
std::pair<double, double> compute_curvature(const std::vector<std::vector<double>>& path);

// Fonction wrapper pour calculer la régularité
double compute_smoothness(const std::vector<std::vector<double>>& path);

// Module pybind11 
PYBIND11_MODULE(metrics_bindings, m) {
    m.doc() = "Liaisons Python pour les métriques MPB"; 
    
    m.def("compute_path_length", &compute_path_length, "Calcule la longueur totale du chemin");
    m.def("compute_curvature", &compute_curvature, "Calcule la courbure maximale et normalisée");
    m.def("compute_smoothness", &compute_smoothness, "Calcule la régularité du chemin");
}

} // namespace metrics