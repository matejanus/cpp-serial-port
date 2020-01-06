#include <iostream>
#include "SerialDriver.hpp"
#include <condition_variable>

using std::cout;
using std::cin;


int main()
{   
    cout<<"serial parser:\n";
    std::condition_variable condVar;
    std::mutex m_mutex;
    std::unique_lock<std::mutex> mlock(m_mutex);
    auto sd = SerialDriver(condVar); 
    sd.start();
    while(1){
        condVar.wait(mlock);
        std::cout<<sd.getMsg();
    }
    return 0;
}