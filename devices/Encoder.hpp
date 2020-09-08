/*
 * @file:    Encoder.hpp
 * @author:  pdsherman
 * @date:    March 2020
 * @brief:   Obejct to communicate to encoder controller
 */

#pragma once

#include <stdint.h>
#include <string>

/// Interface for an encoder reading device.
class Encoder {
public:
  /// Constructor
  /// @param [in] i2c_dev Name for i2c peripheral to use
  /// @param [in] addr Address for device reading the encoder
  /// @param [in] cnt_per_rev Number of encoder counts per revolution
  Encoder(const std::string &i2c_dev, const uint8_t addr, const double cnt_per_rev = 1600.0);

  /// Destructor
  ~Encoder(void);

  /// Open connection to peripherial
  /// @return True if connection successful
  bool connect(void);

  /// Get the position of the encoder
  /// Will convert counts to radians and subtract offset if set by user
  /// @return Position in radians
  double position(void);

  /// Set an offset in softare for convenience
  /// when calling position method
  /// @param [in] offset_rad Offset in radians
  void set_offset(const double offset_rad);

  /// Get the current position offset
  /// @return Offset set by user
  double get_offset(void) const;

  /// Get raw count value from encoder
  /// @note Ignores offset
  /// @return Encoder position in counts
  int32_t raw_count(void);

  /// Set the current position of the encoder sensor to 0
  void zero_position(void);

private:

  /// Close the connection to device
  /// @return True if connection was open and successfully closed
  bool close_port(void);

  /// Is the connection currently open
  bool _is_open;

  /// Name for i2c peripheral
  std::string _dev_name;

  /// Slave address for device
  uint8_t _slave_addr;

  /// File descriptor for connection
  int _file_des;

  /// Offset to subtract (in radians) from position when calling position method
  double _offset_radians;

  /// Scaling encoder counts to radians
  const double _cnt_to_rad;

  /// Constant PI
  static constexpr double pi  = 3.14159;

};
