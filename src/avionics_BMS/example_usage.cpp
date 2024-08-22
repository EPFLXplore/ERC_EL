#include "CppLinuxSerial/SerialPort.hpp"
#include "BMS_control.hpp"
using namespace mn::CppLinuxSerial;

// Example usage
int main() {
    SerialPort serialPort("/dev/ttyUSB0", BaudRate::B_9600, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE, HardwareFlowControl::ON, SoftwareFlowControl::OFF);
    BMSControl TinyBMS(serialPort);
    TinyBMS.init_BMS_comm();

    // reset BMS
    TinyBMS.reset_BMS(BMS);

    // get total voltage
    std::vector<uint8_t> pack_voltage = TinyBMS.send_read_command(PACK_VOLTAGE);
    std::cout << "PACK_VOLTAGE: " << pack_voltage[0] << std::endl;

    // get temperature readings
    std::vector<uint8_t> temps = TinyBMS.send_read_command(TEMPERATURE);
    for(uint8_t val : temps)
        std::cout << (int)val << std::endl;

    return 0;
}