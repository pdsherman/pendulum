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
#include <array>

namespace util {

/// Given a set of X and Y where Y=f(X), use linear interpolation to estimate f(x_n)
/// @note Assumes set X is increasing monotonically increasing and size of X & Y is the same
/// @param [in] x Set of X values
/// @param [in] y Set of Y corresponding to y=f(x)
/// @param [in] x_n Point on X axis to get estimate of f(x)
/// @return Interpolated value of f(x_n)
double interpolate(const std::vector<double> &x, const std::vector<double> &y, const double x_n);

/// Parse a CSV file to a standard C++ data structure.
/// @note Assumes all data is of type double.
/// @param [in] csv_file Data file in csv format
/// @return Map with key as column header and data as value
std::map<std::string, std::vector<double>> get_data_from_csv(const std::string& csv_file);

/// Convenience function to add two arrays.
/// @param [in] a First array
/// @param [in] b Second array
/// @return Value of array addition
template <typename T, int N>
std::array<T, N> add_arrays(const std::array<T, N> &a, const std::array<T, N> &b)
{
  std::array<T, N> c;
  for(size_t ii = 0; ii < N; ++ii)
    c[ii] = a[ii] + b[ii];
  return c;
}

} // namespace util
