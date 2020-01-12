#pragma once
#include <thread>
#include <iostream>
#include <queue>
#include <string>
#include <mutex>
#include <condition_variable>
#include "SerialPort.hpp"

class SerialDriver
{

public:
    void start();
    SerialDriver(std::condition_variable &var);
    SerialDriver(const std::condition_variable &&) = delete;
    ~SerialDriver();
    std::string getMsg();
    int getMessageQueueSize(){return msgQueue.size();}
private:
    void ThreadMain();

    std::thread m_thread;
    bool m_running;
    std::queue<std::string> msgQueue;
    std::mutex mutex;
    std::condition_variable &condVar;
    SerialPort m_port;
};
