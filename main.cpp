#include <iostream>
#include "SerialDriver.hpp"

using std::cout;
using std::cin;


int main()
{
    cout<<"enter something:\n";
    auto sd = SerialDriver(); 
    sd.start();
    while(1){}
    return 0;
}