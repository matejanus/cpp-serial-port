#include "SerialDriver.hpp"

SerialDriver::SerialDriver(std::condition_variable &var) : m_thread(), m_running(true), condVar(var), m_port(SerialPort("/dev/ttyUSB0", SerialPortParams::BaudRate::B_115200))
{
}

SerialDriver::~SerialDriver()
{
    m_running = false;
    if (m_thread.joinable())
    {
        m_thread.join();
    }
}

std::string SerialDriver::getMsg()
{
    if (!msgQueue.empty())
    {
        auto tmp = msgQueue.front();
        msgQueue.pop();
        return "msg: " + tmp;
    }
    return std::string();
}

void SerialDriver::start()
{
    m_thread = std::thread(&SerialDriver::ThreadMain, this);
}

void SerialDriver::ThreadMain()
{
    // auto sp = SerialPort("/dev/ttyUSB0", SerialPortParams::BaudRate::B_115200);
    while (m_running)
    {
        // Do something useful, e.g:
        // auto data = sp.readPort();
        // std::cout<<data;
        // msgQueue.push(&data);
        std::lock_guard<std::mutex> lockGuard(mutex);
        msgQueue.push(m_port.readPort());
        condVar.notify_all();
        // std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}
