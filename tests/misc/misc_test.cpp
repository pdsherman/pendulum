
#include <libs/util/util.hpp>
#include <libs/function_timer/FunctionTimer.hpp>

#include <array>
#include <cmath>
#include <iostream>
#include <map>
#include <vector>

void test1(void)
{
  std::map<std::string, std::vector<double>> data;

  data["X"] = {8.0, 3.2, 0.9, 1.5, -0.9, 4.5};
  data["Y"] = {5.5, 7.8, 1.21, 2.25, 5.49, 20.30, -0.9};
  data["Z"] = {4.5, 2.8, 2.21, 3.25, 1.49, 33.30, -12.9};

  const std::string file = "/home/pdsherman/projects/pendulum/catkin_ws/src/pendulum/data/write_test.csv";
  util::write_data_to_csv(file, data);
}

int main(int argc, char *argv[])
{
  return 0;
}
