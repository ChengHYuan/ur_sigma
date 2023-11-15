//
// Created by jiy on 2021/11/23.
//

#ifndef SRC_CONFIDENCE_H
#define SRC_CONFIDENCE_H
#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <iostream>
#include "dmp/dmp.h"
static double DTWDistance(const std::vector<std::vector<double>>& temp, const std::vector<std::vector<double>>& sign);
static double DTWDistance(const std::vector<dmp::DMPPoint>& temp, const std::vector<dmp::DMPPoint>& sign);

std::vector<double> Confidances(const std::vector<std::vector<std::vector<double>>>& temps, const std::vector<std::vector<double>>& sign);
std::vector<double> Confidances(const std::vector<std::vector<dmp::DMPPoint>>& temps, const std::vector<dmp::DMPPoint>& sign);

#endif //SRC_CONFIDENCE_H
