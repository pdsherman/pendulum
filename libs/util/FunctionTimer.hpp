/*
 * File:    FunctionTimer.hpp
 * Author:  pdsherman
 * Date:    Sept. 2020
 */

#pragma once

#include <vector>
#include <chrono>

namespace util {
/// Class getting the average time for an event such as a function
/// being called during a loop. Class can be used as follows
///
/// FunctionTimer tmr
/// Loop:
///    tmr.start()
///    ... Run code to time
///    tmr.stop()
/// End Loop
/// avg_time = tmr.average_us()
class FunctionTimer
{
public:
  /// Constructor
  FunctionTimer(void);

  /// Default Destructor
  ~FunctionTimer(void) = default;

  /// Start timing an event
  void start(void);

  /// Stop timing and save time for event
  void stop(void);

  /// Reset to clear all previously timed events to start over
  void reset(void);

  /// Get the average time of all timed events since
  /// object was contructed or last reset
  /// @return Average time (micro-seconds) of all timed events
  double average_us(void) const;

  /// Get a copy of all the recorded times
  /// @return Copy of vector containing times
  std::vector<double> all_timed_events_us(void) const;

private:

  /// Saved times of events
  std::vector<double> _t;

  /// Start time of event
  std::chrono::time_point<std::chrono::high_resolution_clock> _start;

  /// Is timer currently timing an event
  bool _active;
};
} //namespace util
