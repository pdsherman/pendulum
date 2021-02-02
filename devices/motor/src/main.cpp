


#include <libserial/SerialPort.h>

#include <string>
#include <iostream>

int main(int argc, char* argv[])
{

  std::string port_addr = "/dev/serial0";

  SerialPort port();

  port.Open(port_addr);

  if(!port.IsOpen()) {
    std::cout << "Error: Unable to Open" << std::endl;
    return -1;
  }

  port.SetBaudRate(BaudRate::BAUD_9600);
  port.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
  port.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
  port.SetParity(Parity::PARITY_NONE);
  port.SetStopBits(StopBits::STOP_BITS_1);


  std::string msg = "g f0x41";
  msg += '\r';

  port.Write(msg);

  std::string rx_msg = "";
  port.ReadLine(rx_msg, '\r', 25);

  std::cout << "Received: " << rx_msg << std::endl;

  return 0;
}
