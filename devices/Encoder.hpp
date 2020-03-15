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
  Encoder(const std::string &i2c_dev, const uint8_t addr);

  /// Destructor
  ~Encoder(void);

  /// Open connection to peripherial
  /// @return True if connection successful
  bool connect(void);

  /// Read the position of the encoder
  /// @return Encoder position in counts
  int32_t position(void);

  /// Set the current positino of the encoder to 0
  void zero_position(void);

  void all_data(int32_t &pos, int16_t &sldr);

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
};
