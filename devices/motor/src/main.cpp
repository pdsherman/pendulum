


#include <libserial/SerialPort.h>
#include <external_libs/copley/CopleyCommandBuilderAscii.hpp>
#include <external_libs/copley/CopleyBus.hpp>
#include <external_libs/copley/CopleyParameter.hpp>

#include <string>
#include <iostream>

using namespace LibSerial;

int main(int argc, char* argv[])
{

  std::string serial_port_addr = "/dev/serial0";
  CopleyBus bus;
  if(bus.connect(serial_port_addr)) {
    CopleyCommandBuilderAscii cmd_builder(0);
    CopleyCommand cmd = cmd_builder.build_get_command(CopleyParameter::kMotorModel,
      CopleyParameter::kMotorModel.location);
    bus.write_command(cmd);
  }


  return 0;
}
