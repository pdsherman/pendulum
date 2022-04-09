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


/// Write data from a c++ container to SQLite database
/// @return True if write to database is successfull
bool write_data_to_db(const std::string &table,
        const std::string &test,
        const std::vector<std::string> &header,
        const std::vector<std::vector<double>> &data);

} // namespace util
