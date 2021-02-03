

#include <external_libs/copley/CopleyCommandBuilderAscii.hpp>
#include <external_libs/copley/CopleyCommandBuilderBinary.hpp>
#include <external_libs/copley/CopleyResponseBinary.hpp>
#include <external_libs/copley/CopleyResponseAscii.hpp>
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
    CopleyCommandBuilderBinary cmd_builder(0);
    CopleyCommand cmd = cmd_builder.build_get_command(CopleyParameter::kMotorModel,
      CopleyParameter::kMotorModel.location);

    std::unique_pre<CopleyResponse> resp = bus.write_command(cmd);
    std::cout << "Response: " << resp->to_string() << std::endl;

  }


  return 0;
}
