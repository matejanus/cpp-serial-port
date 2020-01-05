#pragma once
#include <thread>
#include <iostream>
#include <chrono>
#include <queue>
#include <string>
#include <mutex>

class SerialDriver
{

public:
    void start();
    SerialDriver();
    ~SerialDriver();
    std::string getMsg();
private:
    void ThreadMain();

    std::thread m_thread;
    bool m_running;
    std::queue<std::string> msgQueue;
    std::mutex mutex;
};
