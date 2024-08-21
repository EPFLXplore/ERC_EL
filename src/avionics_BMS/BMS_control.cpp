/*
 * Author:    Josef Thomas, Tomas
 * Date:      March 21, 2024
 * Purpose:   Uart Communication driver for tiny BMS: https://enepaq.com/product/battery-management-system-bms/
 */

#include "CppLinuxSerial/SerialPort.hpp"

using namespace mn::CppLinuxSerial;

const static uint16_t crcTable[256] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

/**** BMS ADDRESS 8 bits ****/
#define ADDR 0xAA

/**** BMS COMMANDS: 8 bits ****/
#define RESET       0x12
#define NEW_EVENTS  0x11 // get all new events: packet length (bytes) in 3rd byte of response
#define ALL_EVENTS  0x12 // get all events: packet length (bytes) in 3rd byte of response

#define PACK_VOLTAGE 0x14 //sum of voltage of all cells
#define PACK_CURRENT 0x15 //sum of current of all cells

#define MAX_CELL_VOLT 0x16 //max cell voltage configuration
#define MIN_CELL_VOLT 0x17 //min cell voltage configuration

#define STATUS              0x18 //get status of BMS: see status ids below
#define LIFETIME_COUNTER    0x19 //get lifetime counter of BMS
#define TEMPERATURE         0x1B //get temperatures of BMS: 16 bits; internal_temp, ext_temp1, ext_temp2

#define CELLS_VOLTAGE 0x1C //get voltage of all cells: 16 bits per cell, length in bytes defined in 3rd byte

#define VERSION 0x1E //get version of BMS: 16 bits; length in bytes defined in 3rd byte of response

#define READ_INDIVIDUAL_REGISTERS   0x09 //get individual registers: 16 bits per register, length in bytes defined in 3rd byte of request
#define WRITE_INDIVIDUAL_REGISTERS  0x0D //set individual registers: 16 bits per register, length in bytes defined in 3rd byte of request

/**** BMS RESET OPTIONS: 8 bits ****/
#define EVENTS  0x01
#define Stats   0x02
#define BMS     0x05

/**** BMS RESPONSES: 8 bits ****/
#define NACK_UART_ERROR 0x00
#define ACK_UART        0x01

/*** SPECIAL_ERROR types ***/
#define CMD_ERROR 0x00
#define CRC_ERROR 0x01

/**** STATUS IDS: 16 bits ****/
#define CHARGING        0x91
#define CHARGED         0x92
#define DISCHARGING     0x93
#define REGEN           0x96
#define IDLE            0x97
#define FAULT           0x9B

/**** Data positions ****/
#define DATA_ADDR   0
#define DATA_ACK    1
#define DATA_PL     2
#define DATA_CMD    2

//check if command has a dynamic length response
#define IS_DYNAMIC_LEN(command) (command == NEW_EVENTS || command == ALL_EVENTS || command == CELLS_VOLTAGE || command == VERSION)

void init_BMS_comm(SerialPort& serialPort);

// API
bool reset_BMS(SerialPort& serialPort, uint8_t option);
bool send_command(SerialPort& serialPort, uint8_t command);
std::vector<uint8_t> send_read_command(SerialPort& serialPort, uint8_t command);
bool write_individual_registers(SerialPort& serialPort, std::vector<uint16_t> addresses, std::vector<uint16_t> data);
std::vector<uint8_t> read_individual_registers(SerialPort& serialPort, std::vector<uint16_t> addresses);

// WRITE FUNCTIONS
uint16_t write_1_byte(SerialPort& serialPort, uint8_t data);
uint16_t write_n_bytes(SerialPort& serialPort, uint8_t command, std::vector<uint16_t> data);

// READ FUNCTION
std::vector<uint8_t> read_n_bytes(SerialPort& serialPort, uint8_t command, uint16_t crc);

// UTILS
bool check_packet_error(std::vector<uint8_t> response, uint8_t command, uint16_t crc);
uint16_t append_crc(std::vector<uint8_t>& packet);
inline uint16_t CRC16(const uint8_t* data, uint16_t length);

struct report{
    uint8_t event;
    uint8_t length;
    uint8_t* data;
};

// Example usage
int main() {
    SerialPort serialPort("/dev/ttyUSB0", BaudRate::B_9600, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE, HardwareFlowControl::ON, SoftwareFlowControl::OFF);
    init_BMS_comm(serialPort);

    // reset BMS
    reset_BMS(serialPort, BMS);

    // get total voltage
    std::vector<uint8_t> pack_voltage = send_read_command(serialPort, PACK_VOLTAGE);
    std::cout << "PACK_VOLTAGE: " << pack_voltage[0] << std::endl;

    // get temperature readings
    std::vector<uint8_t> temps = send_read_command(serialPort, TEMPERATURE);
    for(uint8_t val : temps)
        std::cout << (int)val << std::endl;

    serialPort.Close();
    return 0;
}

void init_BMS_comm(SerialPort& serialPort){
    //init serial port
    serialPort.SetTimeout(1000); // Block when reading for 1000ms
    serialPort.Open();
}

/**** API: functions that are recommended to be used ****/

// reset BMS with option(see above): ADDR, RESET, option, CRC(LSB), CRC(MSB)
bool reset_BMS(SerialPort& serialPort, uint8_t option){
    std::vector<uint8_t> packet = {ADDR, RESET, option};
    uint16_t crc = append_crc(packet);
    serialPort.WriteBinary(packet);
    return (read_n_bytes(serialPort, RESET, crc)[0] != -1);
}

bool send_command(SerialPort& serialPort, uint8_t command){
    uint16_t crc = write_1_byte(serialPort, command);

    std::vector<uint8_t> response;
    serialPort.ReadBinary(response);
    if(check_packet_error(response, command, crc)){ //command not properly acknowledged
        return false;
    }
    return true;
}

std::vector<uint8_t> send_read_command(SerialPort& serialPort, uint8_t command){
    uint16_t crc = write_1_byte(serialPort, command);
    return read_n_bytes(serialPort, command, crc);
}

// write data to specific registers
bool write_individual_registers(SerialPort& serialPort, std::vector<uint16_t> addresses, std::vector<uint16_t> data){
    if(addresses.size() != data.size()){ // error lengths don't match
        return false;
    }
    std::vector<uint16_t> new_data;
    for(int i = 0; i < addresses.size(); i++){
        new_data.push_back(addresses[i]);
        new_data.push_back(data[i]);
    }

    uint16_t crc = write_n_bytes(serialPort, WRITE_INDIVIDUAL_REGISTERS, new_data);
    if (crc != -1){ //no crc error
        std::vector<uint8_t> response;
        serialPort.ReadBinary(response);

        if(!check_packet_error(response, WRITE_INDIVIDUAL_REGISTERS, crc))
            return true; //properly acknowledged
    }
    return false; // not properly acknowledged
}

std::vector<uint8_t> read_individual_registers(SerialPort& serialPort, std::vector<uint16_t> addresses){
    uint16_t crc = write_n_bytes(serialPort, READ_INDIVIDUAL_REGISTERS, addresses);

    return read_n_bytes(serialPort, CELLS_VOLTAGE, crc);
}

/**** Write functions ****/

// write 1 byte command to BMS: ADDR, data, CRC(LSB), CRC(MSB)
uint16_t write_1_byte(SerialPort& serialPort, uint8_t data){
    std::vector<uint8_t> packet = {ADDR, data};
    uint16_t crc = append_crc(packet);
    serialPort.WriteBinary(packet);
    return crc;
}

// write n(max (2^5-1)) bytes to BMS: ADDR, data, CRC(LSB), CRC(MSB)
uint16_t write_n_bytes(SerialPort& serialPort, uint8_t command, std::vector<uint16_t> data){
    if(data.size() > 2^5 - 1){ // too much data to be written
        //ERROR: print error?
        return -1;
    }
    std::vector<uint8_t> packet = {ADDR, command, (uint8_t)data.size()};
    // Convert each uint16_t element in data to two uint8_t elements and append to packet
    for (uint16_t value : data) {
        packet.push_back(value & 0xFF); // LSB
        packet.push_back(value >> 8);  // MSB
    }
    uint16_t crc = append_crc(packet);
    serialPort.WriteBinary(packet);
    return crc;
}

/**** Read functions ****/

// read a variable length packet from BMS based on command
std::vector<uint8_t> read_n_bytes(SerialPort& serialPort, uint8_t command, uint16_t crc){
    //read response
    std::vector<uint8_t> response;
    serialPort.ReadBinary(response);

    if(check_packet_error(response, command, crc)){ //if error return empty vector
        return {};
    }

    std::vector<uint8_t> received_data;
    if(IS_DYNAMIC_LEN(command))
        received_data = std::vector<uint8_t>(response.begin() + DATA_PL, response.begin() + 2 + response[DATA_PL]); //PL bytes stored after 3rd bytes
    else
        received_data = std::vector<uint8_t>(response.begin() + 2, response.end() - 2); //Data stored between 3rd and 2nd last bytes

    return received_data;
}

/**** UTILS ****/

// return true if error in packet
bool check_packet_error(std::vector<uint8_t> response, uint8_t command, uint16_t crc){
    if(response[DATA_ADDR] != ADDR){
        //ERROR: print error?
        return true;
    }
    if(IS_DYNAMIC_LEN(command)){ //if dynamic then PL acts as ACK
        if(response[DATA_PL] != response.size() - 5){ //PL(payload) = packetLen - 5 bytes(header, command, crc)
            return true;
        }

    }
    else if(response[DATA_ACK] != ACK_UART){ //normal error: ACK not respected by BMS
        return true;
    }
    if(response[DATA_ACK] == NACK_UART_ERROR){ //special error
        if(response[DATA_CMD] == CMD_ERROR){
            //ERROR: print error?
        }
        if(response[DATA_CMD] == CRC_ERROR){
            //ERROR: print error?
        }
        return true;
    }
    if(response[DATA_CMD] != command){ //unexpected command responsed
        //ERROR: print error?
        return true;
    }

    //check crc
    uint16_t crc_response = ((response.size() - 1) << 8) | (response.size() - 2); //last 2 bytes of packet are crc
    if(crc != crc_response){
        //ERROR: print error?
        return true;
    }
    return false; // no error
}

uint16_t append_crc(std::vector<uint8_t>& packet){
    // add 0s for crc
    packet.push_back(0);
    packet.push_back(0);

    uint16_t crc = CRC16(packet.data(), packet.size());
    *(packet.end() - 2) = crc & 0xFF; //LSB in 2nd last byte
    *(packet.end() - 1) = crc >> 8; //MSB in last byte
    return crc;
}

// function defined in datasheet
inline uint16_t CRC16(const uint8_t* data, uint16_t length){
    uint8_t tmp;
    uint16_t crcWord = 0xFFFF;
    while (length--){
        tmp = *data++ ^ crcWord;
        crcWord >>= 8;
        crcWord ^= crcTable[tmp];
    }
    return crcWord;
}

