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
class EncoderBoard {
public:

  /// Different modes for the encoder board. Sets what
  /// infomation the board will return on i2c read.
  enum class Mode : uint8_t {
    OnlyOne = 1,  ///< Board returns position of Encoder #1
    OnlyTwo = 2,  ///< Board returns position of Encoder #2
    Both = 3      ///< Board returns position of both encoders.
  };

  /// Specifies encoder selection for some class methods.
  enum class Encoder{One, Two};

  /// Constructor
  /// @param [in] i2c_dev Name for i2c peripheral to use
  /// @param [in] addr Address for device reading the encoder
  /// @param [in] cnt_per_rev Number of encoder counts per revolution
  EncoderBoard(const std::string &i2c_dev, const uint8_t addr, const double cnt_per_rev = 1600.0);

  /// Destructor
  ~EncoderBoard(void);

  /// Open connection to peripherial
  /// @return True if connection successful
  bool connect(void);

  /// Check if I2C connection is established
  /// @return True if connection is open
  bool is_connected(void) const;

  /// Set the mode of the encoder board.
  ///
  void set_mode(cosnt Mode mode) const;
  
  /// Get the position of the encoder
  /// Will convert counts to radians and subtract offset if set by user
  /// @return Position in radians
  double position(void);
  std::array<double, 2> position(void);

  /// Set an offset in softare for convenience
  /// when calling position method
  /// @param [in] encoder Which encoder offset to set
  /// @param [in] offset_rad Offset in radians
  void set_offset(const Enocder encdr, const double offset_rad);

  /// Get the current position offset
  /// @param [in] encdr Which encoder offset to get
  /// @return Offset set by user
  double get_offset(const Enocder encdr) const;

  /// Get raw count value from encoder
  /// @note Ignores offset
  /// @return Encoder position in counts
  int32_t raw_count(void);
  std::array<int32_t, 2> raw_count(void);

  /// Set the current position of the encoder sensor to 0
  void zero_position(void);

private:

  /// Is the connection currently open
  bool _is_open;

  /// Name for i2c peripheral
  std::string _dev_name;

  /// Slave address for device
  uint8_t _slave_addr;

  /// File descriptor for connection
  int _file_des;

  /// Current setting for the encoder board mode
  Mode _mode;

  /// Offset to subtract (in radians) from position when calling position method
  double _offset_radians_1;
  double _offset_radians_2;

  /// Scaling encoder counts to radians
  const double _cnt_to_rad_1;
  const double _cnt_to_rad_2;

  /// Constant PI
  static constexpr double pi  = 3.14159;

};
