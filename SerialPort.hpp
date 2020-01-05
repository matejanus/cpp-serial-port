#pragma once

#include <string>
#include <fstream> // For file I/O (reading/writing to COM port)
#include <sstream>
#include <termios.h> // POSIX terminal control definitions (struct termios)
#include <vector>
#include <iostream>
#include <stdio.h>  // Standard input/output definitions
#include <unistd.h> // UNIX standard function definitions
#include <cstring>
#include <fcntl.h> 
#include <array>

#include "Exception.hpp"

namespace SerialPortParams
{
enum class BaudRate
{
    B_9600,
    B_38400,
    B_57600,
    B_115200,
    CUSTOM
};

enum class State
{
    CLOSED,
    OPEN
};
} // namespace SerialPortParams

class SerialPort
{

public:
    SerialPort() = delete;
    SerialPort(std::string device, SerialPortParams::BaudRate baudRate);
    SerialPort(std::string device, SerialPortParams::BaudRate baudRate, int timeout);
    virtual ~SerialPort();

    std::string readPort();
    void writePort(const std::string& data);
    void openPort();

    void setDevice(const std::string &device);
    void setBaudRate(SerialPortParams::BaudRate baudRate);
    void setTimeout(int32_t timeout_ms);
    void setEcho(bool value);

private:
    void closePort();
    void configureTermios();

    std::string m_device;
    SerialPortParams::BaudRate m_baudRate;
    int m_fileDesc;
    SerialPortParams::State m_state;
    int m_timeout;
    bool echoEnabled;
};
