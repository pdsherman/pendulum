/*
 * @file:    Encoder.hpp
 * @author:  pdsherman
 * @date:    March 2020
 * @brief:   Obejct to communicate to encoder controller
 */

#pragma once

#include <stdint.h>
#include <string>

class Encoder {
public:

    Encoder(const std::string &i2c_dev, const uint8_t addr);

    ~Encoder(void);

    bool connect(void);

    int32_t position(void);

    void zero_position(void);

private:

  bool close_port(void);

  bool _is_open;

  std::string _dev_name;
  uint8_t _slave_addr;
  int _file_des;

};
