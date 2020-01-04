#pragma once

// System includes
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

class SerialPortException : public std::runtime_error {

    public:
        SerialPortException(const char *file, int line, const std::string &arg) :
                std::runtime_error(arg) {
            m_msg = std::string(file) + ":" + std::to_string(line) + ": " + arg;
        }

        ~SerialPortException() throw() {}

        const char *what() const throw() override {
            return m_msg.c_str();
        }

    private:
        std::string m_msg;
    };
