/*
 * @file:   log_util.hpp
 * @author: pdsherman
 * @date:   April 2022
 * @brief:  Assortment of useful funtions with logging stuff
 */

#pragma once

#include <vector>
#include <string>
#include <map>

namespace util {

/// Read data from a SQLite database and insert into a useful c++ container
/// @warning assumes certain structure of data (first column is a STRING testname, all others DOUBLE)
/// @param [in] select_stmt The SQLite select statement to get data
/// @return Map of column-names to the actual data
std::map<std::string, std::vector<double>> read_data_from_db(const std::string &select_stmt);

} // namespace util
