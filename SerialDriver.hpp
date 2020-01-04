#pragma once
#include <thread>
#include <iostream>
#include <chrono>


class SerialDriver
{

public:
    void start();
    SerialDriver();
    ~SerialDriver();
private:
    void ThreadMain();

    std::thread m_thread;
    bool m_running;
};
