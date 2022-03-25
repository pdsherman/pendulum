
#include <hardware/encoder/EncoderBoard.hpp>

#include <iostream>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

EncoderBoard::EncoderBoard(const std::string &i2c_dev, const uint8_t addr, const double cnt_per_rev) :
  _is_open(false),
  _dev_name(i2c_dev),
  _slave_addr(addr),
  _file_des(-1),
  _mode(Mode::Both),
  _offset_radians_1(0.0),
  _offset_radians_2(0.0),
  _cnt_to_rad_1(2*pi/cnt_per_rev),
  _cnt_to_rad_2(2*pi/cnt_per_rev)
{
}

EncoderBoard::~EncoderBoard(void)
{
  if(_is_open) { ::close(_file_des); }
}

bool EncoderBoard::connect(void)
{
  _file_des = ::open(_dev_name.c_str(), O_RDWR);
  if(_file_des < 0 || ::ioctl(_file_des, I2C_SLAVE, _slave_addr) < 0) {
    _is_open = false;
  } else {
    _is_open = true;
    set_mode(_mode);
  }

  return _is_open;
}

bool EncoderBoard::is_connected(void) const
{
  return _is_open;
}

void EncoderBoard::set_mode(const Mode mode)
{
  _mode = mode;
  uint8_t data = static_cast<uint8_t>(mode);
  ::write(_file_des, &data, 1);
}

std::array<double, 2> EncoderBoard::position(void)
{
  std::array<int32_t, 2> raw = raw_count();

  std::array<double, 2> pos = {
    static_cast<double>(raw[0])*_cnt_to_rad_1 + _offset_radians_1,
    static_cast<double>(raw[1])*_cnt_to_rad_2 + _offset_radians_2,
  };

  return pos;
}

double EncoderBoard::position_single(void)
{
  if(_mode == Mode::OnlyOne || _mode == Mode::Both)
    return static_cast<double>(raw_count_single())*_cnt_to_rad_1 + _offset_radians_1;
  else
    return static_cast<double>(raw_count_single())*_cnt_to_rad_2 + _offset_radians_2;
}

void EncoderBoard::set_offset(const Encoder encdr, const double offset_rad)
{
  if(encdr == Encoder::One)
    _offset_radians_1 = offset_rad;
  else
    _offset_radians_2 = offset_rad;
}

double EncoderBoard::get_offset(const Encoder encdr) const
{
  if(encdr == Encoder::One)
    return _offset_radians_1;
  return _offset_radians_2;
}

int32_t EncoderBoard::raw_count_single(void)
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

std::array<int32_t, 2> EncoderBoard::raw_count(void)
{
  std::array<int32_t, 2> raw = {0, 0};
  if(_is_open) {
    int num = ::read(_file_des, raw.data(), 8);
    if(num != 8) {
      std::cout << "Error On Read" << std::endl;
    }
  }

  return raw;
}

void EncoderBoard::zero_position(void)
{
  if(_is_open){
    uint8_t data[1];
    data[0] = kZeroCommand;
    ::write(_file_des, data, 1);
  }
}
