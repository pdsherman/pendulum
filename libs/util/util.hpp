/*
 * @file:   util.hpp
 * @author: pdsherman
 * @date:   April 2020
 * @brief:  Assortment of useful funtions
 */

#pragma once

#include <termios.h>

#include <vector>
#include <iomanip>
#include <map>
#include <string>
#include <array>
#include <sstream>

namespace util {

/// Given a set of X and Y where Y=f(X), use linear interpolation to estimate f(x_n)
/// @note Assumes set X is increasing monotonically increasing and size of X & Y is the same
/// @param [in] x Set of X values
/// @param [in] y Set of Y corresponding to y=f(x)
/// @param [in] x_n Point on X axis to get estimate of f(x)
/// @return Interpolated value of f(x_n)
double interpolate(const std::vector<double> &x, const std::vector<double> &y, const double x_n);


/// Read all the lines from a file and insert into c++ container
/// @param [in] filename Name of file to read
/// @return Container filled with each line from file, in order read from file.
std::vector<std::string> read_text_from_file(const std::string& filename);

/// Parse a CSV file to a standard C++ data structure.
/// @note Assumes all data is of type double.
/// @param [in] csv_file Data file in csv format
/// @return Map with key as column header and data as value
std::map<std::string, std::vector<double>> read_data_from_csv(const std::string& csv_file);

/// Write data to a file in CSV format
/// @param [in] csv_file File to write
/// @param [in] data Map of data. Key will used as header column names
void write_data_to_csv(const std::string &csv_file, std::map<std::string, std::vector<double>> data);

/// Converts POSIX speed_t to a baud rate, as an integer.  The values of the
/// constants for speed_t are not themselves portable.
int speed_to_baud(speed_t speed);

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

/// Cast an enum to its underlying type.
/// @param [in] e The enumerated value to cast.
/// @return e, cast to its underlying type.
template <typename E> constexpr auto to_underlying(E e) -> typename std::underlying_type<E>::type
{
	return static_cast<typename std::underlying_type<E>::type>(e);
}

/// Translate a byte to a formatted hexidecimal string 0x__.
/// @param [in] byte The byte to translate.
/// @return formatted hexidecimal string.
template <typename T> std::string int_to_hex(T i)
{
	std::stringstream stream;
	stream << "0x" << std::setfill('0') << std::setw(sizeof(T) * 2) << std::uppercase << std::hex << +i;
	return stream.str();
}

} // namespace util
