#include "SerialDriver.hpp"
#include "SerialPort.hpp"

SerialDriver::SerialDriver() : m_thread(), m_running(false)
{
}

SerialDriver::~SerialDriver()
{
    m_running = true;
    if (m_thread.joinable())
    {
        m_thread.join();
    }
}
void SerialDriver::start()
{
    m_thread = std::thread(&SerialDriver::ThreadMain, this);
}

void SerialDriver::ThreadMain()
{   
    int i = 0; 
    auto sp = SerialPort("/dev/ttyUSB0", SerialPortParams::BaudRate::B_115200);
    while (!m_running)
    {
        // Do something useful, e.g:
        auto data = sp.readPort();
        std::cout<<data;
        std::cout << "hi there " << i <<std::endl;
        i++;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}