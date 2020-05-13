/*
 * @file:   util.hpp
 * @author: pdsherman
 * @date:   April 2020
 * @brief:  Assortment of useful funtions
 */

#pragma once

#include <vector>
#include <map>
#include <string>

double interpolate(const std::vector<double> &x, const std::vector<double> &y, const double x_n);

std::map<std::string, std::vector<double>> get_data_from_csv(const std::string& csv_file);
