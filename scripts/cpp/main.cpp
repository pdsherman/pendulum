/*
  @file:    control_base.cpp
  @author:  pdsherman
  @date:    April. 2022
*/

#include <libs/util/log_util.hpp>

#include <iostream>

int main(int argc, char *argv[])
{
  std::string table_name = "EncoderOneTest";
  std::string test_name  = "real_03";

  std::string cmd = "SELECT * FROM " + table_name +
    " WHERE test_name = '" + test_name + "' ORDER BY test_time";

  std::map<std::string, std::vector<double>> data = util::read_data_from_db(cmd);

  for(const auto &d : data) {
    std::cout << d.first << std::endl;
    for(int i = 0; i < 10; ++i)
      std::cout << d.second[i] << std::endl;
  }

  return 0;
}
