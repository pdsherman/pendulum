/*
 * File:    FunctionTimer.hpp
 * Author:  pdsherman
 * Date:    Sept. 2020
 */

#include <vector>
#include <chrono>

namespace util {

class FunctionTimer
{
public:
  FunctionTimer(void);

  ~FunctionTimer(void) = default;

  void start(void);

  void stop(void);

  double average_us(void) const;

  void reset(void);

private:
  std::vector<double> _t;

  std::chrono::time_point<std::chrono::high_resolution_clock> _start;
  bool _active;

};

}
