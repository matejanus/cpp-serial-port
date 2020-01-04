#include "SerialPort.hpp"

SerialPort::SerialPort(std::string device, SerialPortParams::BaudRate baudRate) : m_device(device),
                                                                                  m_baudRate(baudRate),
                                                                                  m_fileDesc(-1),
                                                                                  m_state(SerialPortParams::State::CLOSED),
                                                                                  m_timeout(-1),
                                                                                  echoEnabled(false)
{
    openPort();
}

SerialPort::SerialPort(std::string device, SerialPortParams::BaudRate baudRate, int timeout) : m_device(device),
                                                                                               m_baudRate(baudRate),
                                                                                               m_fileDesc(-1),
                                                                                               m_state(SerialPortParams::State::CLOSED),
                                                                                               m_timeout(timeout),
                                                                                               echoEnabled(false)
{
    openPort();
}

SerialPort::~SerialPort()
{
    try
    {
        closePort();
    }
    catch (...)
    {
        // We can't do anything about this!
        // But we don't want to throw within destructor, so swallow
    }
}

void SerialPort::closePort()
{
    if (m_fileDesc != -1)
    {
        auto retVal = close(m_fileDesc);
        if (retVal != 0)
            // THROW_EXCEPT("Tried to close serial port " + device_ + ", but close() failed.");
            m_fileDesc = -1;
    }
    m_state = SerialPortParams::State::CLOSED;
}

void SerialPort::configureTermios()
{   
    if(m_fileDesc == -1)
            throw SerialPortException(__FILE__, __LINE__, __PRETTY_FUNCTION__ + std::string("file descriptor not valid."));
   
    struct termios tty;
    std::memset(&tty, 0, sizeof(tty));

    if (tcgetattr(m_fileDesc, &tty) != 0)
    {
        // Error occurred
        std::cout << "Could not get terminal attributes for \"" << m_device << "\" - " << strerror(errno) << std::endl;
        throw std::system_error(EFAULT, std::system_category());
    }

    tty.c_cflag &= ~PARENB;        // No parity bit is added to the output characters
    tty.c_cflag &= ~CSTOPB;        // Only one stop-bit is used
    tty.c_cflag &= ~CSIZE;         // CSIZE is a mask for the number of bits per character
    tty.c_cflag |= CS8;            // Set to 8 bits per character
    tty.c_cflag &= ~CRTSCTS;       // Disable hadrware flow control (RTS/CTS)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    switch (m_baudRate)
    {
    case SerialPortParams::BaudRate::B_9600:
        cfsetispeed(&tty, B9600);
        cfsetospeed(&tty, B9600);
        break;
    case SerialPortParams::BaudRate::B_38400:
        cfsetispeed(&tty, B38400);
        cfsetospeed(&tty, B38400);
        break;
    case SerialPortParams::BaudRate::B_57600:
        cfsetispeed(&tty, B57600);
        cfsetospeed(&tty, B57600);
        break;
    case SerialPortParams::BaudRate::B_115200:
        cfsetispeed(&tty, B115200);
        cfsetospeed(&tty, B115200);
        break;
    case SerialPortParams::BaudRate::CUSTOM:
        // See https://gist.github.com/kennethryerson/f7d1abcf2633b7c03cf0
        throw std::runtime_error("Custom baud rate not yet supported.");
    default:
        throw std::runtime_error(std::string() + "baudRate passed to " + __PRETTY_FUNCTION__ + " unrecognized.");
    }

    //===================== (.c_oflag) =================//

    tty.c_oflag = 0;       // No remapping, no delays
    tty.c_oflag &= ~OPOST; // Make raw

    //================= CONTROL CHARACTERS (.c_cc[]) ==================//

    // c_cc[VTIME] sets the inter-character timer, in units of 0.1s.
    // Only meaningful when port is set to non-canonical mode
    // VMIN = 0, VTIME = 0: No blocking, return immediately with what is available
    // VMIN > 0, VTIME = 0: read() waits for VMIN bytes, could block indefinitely
    // VMIN = 0, VTIME > 0: Block until any amount of data is available, OR timeout occurs
    // VMIN > 0, VTIME > 0: Block until either VMIN characters have been received, or VTIME
    //                      after first character has elapsed
    // c_cc[WMIN] sets the number of characters to block (wait) for when read() is called.
    // Set to 0 if you don't want read to block. Only meaningful when port set to non-canonical mode

    if (m_timeout == -1)
    {
        // Always wait for at least one byte, this could
        // block indefinitely
        tty.c_cc[VTIME] = 0;
        tty.c_cc[VMIN] = 1;
    }
    else if (m_timeout == 0)
    {
        // Setting both to 0 will give a non-blocking read
        tty.c_cc[VTIME] = 0;
        tty.c_cc[VMIN] = 0;
    }
    else if (m_timeout > 0)
    {
        tty.c_cc[VTIME] = (cc_t)(m_timeout / 100); // 0.5 seconds read timeout
        tty.c_cc[VMIN] = 0;
    }

    //======================== (.c_iflag) ====================//

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    //=========================== LOCAL MODES (c_lflag) =======================//

    // Canonical input is when read waits for EOL or EOF characters before returning. In non-canonical mode, the rate at which
    // read() returns is instead controlled by c_cc[VMIN] and c_cc[VTIME]
    tty.c_lflag &= ~ICANON;                                       // Turn off canonical input, which is suitable for pass-through
    echoEnabled ? (tty.c_lflag | ECHO) : (tty.c_lflag & ~(ECHO)); // Configure echo depending on echo_ boolean
    tty.c_lflag &= ~ECHOE;                                        // Turn off echo erase (echo erase only relevant if canonical input is active)
    tty.c_lflag &= ~ECHONL;                                       //
    tty.c_lflag &= ~ISIG;                                         // Disables recognition of INTR (interrupt), QUIT and SUSP (suspend) characters

    tcflush(m_fileDesc, TCIFLUSH);

    if (tcsetattr(m_fileDesc, TCSANOW, &tty) != 0)
    {
        // Error occurred
        std::cout << "Could not apply terminal attributes for \"" << m_device << "\" - " << strerror(errno) << std::endl;
        throw std::system_error(EFAULT, std::system_category());
    }
}

std::string SerialPort::readPort()
{
    if (m_fileDesc == 0)
    {
        // THROW_EXCEPT("Read() was called but file descriptor (fileDesc) was 0, indicating file has not been opened.");
        throw SerialPortException(__FILE__, __LINE__, __PRETTY_FUNCTION__ + std::string("Read() was called but file descriptor (fileDesc) was 0, indicating file has not been opened.\n"));
    }
    std::array<char, 255> buffer = {0};
    int n = read(m_fileDesc, &buffer[0], buffer.size());

    // Error Handling
    if (n < 0)
    {
        // Read was unsuccessful
        throw std::system_error(EFAULT, std::system_category());
    }
    return std::string(begin(buffer), end(buffer));
}

void SerialPort::openPort()
{
    std::cout << "Attempting to open COM port \"" << m_device << "\"." << std::endl;

    if (m_device.empty())
    {
        throw SerialPortException(__FILE__, __LINE__, __PRETTY_FUNCTION__ + std::string("Attempted to open file when file path has not been assigned to."));
    }

    m_fileDesc = open(m_device.c_str(), O_RDWR);

    if (m_fileDesc == -1)
    {
        throw SerialPortException(__FILE__, __LINE__,std::string("Could not open device ") + m_device + std::string(". Is the device name correct and do you have read/write permission?"));
        std::cout << "Could not open device \n";
    }
    configureTermios();
    std::cout << "COM port opened successfully." << std::endl;
    m_state = SerialPortParams::State::OPEN;
}

void SerialPort::writePort(const std::string &data)
{
    if (m_state != SerialPortParams::State::OPEN)
        throw SerialPortException(__FILE__, __LINE__, __PRETTY_FUNCTION__ + std::string("called but state != OPEN. Please call Open() first."));

    if (m_fileDesc < 0)
    {
        throw SerialPortException(__FILE__, __LINE__, __PRETTY_FUNCTION__ + std::string("called but file descriptor < 0, indicating file has not been opened."));
    }

    int writeResult = write(m_fileDesc, data.c_str(), data.size());

    if (writeResult == -1)
    {
        throw std::system_error(EFAULT, std::system_category());
    }
}

void SerialPort::setDevice(const std::string &device)
{
    m_device = device;
    if (m_state == SerialPortParams::State::OPEN)
        configureTermios();
}
void SerialPort::setBaudRate(SerialPortParams::BaudRate baudRate)
{
    m_baudRate = baudRate;
    if (m_state == SerialPortParams::State::OPEN)
        configureTermios();
}
void SerialPort::setTimeout(int32_t timeout_ms)
{

    if (timeout_ms < -1)
        throw SerialPortException(__FILE__, __LINE__, std::string("timeout_ms provided to ") + __PRETTY_FUNCTION__ + std::string(" was < -1, which is invalid."));
    else if (timeout_ms > 25500)
        throw SerialPortException(__FILE__, __LINE__, std::string("timeout_ms provided to ") + __PRETTY_FUNCTION__ + std::string(" was > 25500, which is invalid."));
    if (m_state == SerialPortParams::State::OPEN)
        throw SerialPortException(__FILE__, __LINE__, __PRETTY_FUNCTION__ + std::string(" called while state == OPEN."));
    m_timeout = timeout_ms;
}
void SerialPort::setEcho(bool value)
{
    echoEnabled = value;
    if (m_state == SerialPortParams::State::OPEN)
        configureTermios();
}
