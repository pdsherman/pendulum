
#include <libs/function_timer/FunctionTimer.hpp>

#include <numeric> // accumulate

namespace util {

FunctionTimer::FunctionTimer(void) : _t(), _start(), _active(false)
{}

void FunctionTimer::start(void)
{
  _start = std::chrono::high_resolution_clock::now();
  _active = true;
}

void FunctionTimer::stop(void)
{
  if(_active) {
    auto stop = std::chrono::high_resolution_clock::now();
    auto dt = std::chrono::duration_cast<std::chrono::nanoseconds>(stop - _start);
    _t.push_back(static_cast<double>(dt.count())/1000.0);
    _active = false;
  }
}

void FunctionTimer::reset(void)
{
  _t.clear();
  _active = false;
}

double FunctionTimer::average_us(void) const
{
  return std::accumulate(_t.begin(), _t.end(), 0.0) / _t.size();
}

std::vector<double> FunctionTimer::all_timed_events_us(void) const
{
  return _t;
}

} // namespace util
