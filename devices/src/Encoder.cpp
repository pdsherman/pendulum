
#include <devices/Encoder.hpp>

#include <iostream>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

Encoder::Encoder(const std::string &i2c_dev, const uint8_t addr) :
  _is_open(false),
  _dev_name(i2c_dev),
  _slave_addr(addr),
  _file_des(-1)
{
}

Encoder::~Encoder(void)
{
  if(_is_open) { close_port(); }
}

bool Encoder::connect(void)
{
  _file_des = ::open(_dev_name.c_str(), O_RDWR);
  if(_file_des < 0 || ::ioctl(_file_des, I2C_SLAVE, _slave_addr) < 0) {
    _is_open = false;
  } else {
    _is_open = true;
  }

  return _is_open;
}

bool Encoder::close_port(void)
{
  return !(_file_des != -1 && ::close(_file_des) < 0);
}

int32_t Encoder::position(void)
{
  int32_t pos = 0;
  if(_is_open) {
    int num = ::read(_file_des, &pos, 4);
    if(num != 4) {
      std::cout << "Error On Read" << std::endl;
    }
  }

  return pos;
}

void Encoder::zero_position(void)
{
  if(_is_open){
    uint8_t data[1];
    data[0] = 0x01;
    ::write(_file_des, data, 1);
  }
}
