#ifndef RS485CONTROLLER_H
#define RS485CONTROLLER_H

#include <vector>
#include <string>
#include <chrono>
#include <iostream>

#ifdef __linux__
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>
#endif

#define RS485_DEFAULT_READ_TIMEOUT 5 // s

enum Rs485Bitrate
{
    RS485_9600 = 9600,
    RS485_19200 = 19200,
    RS485_38400 = 38400,
    RS485_57600 = 57600,
    RS485_115200 = 115200,
    RS485_230400 = 230400,
    RS485_460800 = 460800,
    RS485_1000000 = 1000000,
    RS485_2000000 = 2000000
};

class Rs485Controller
{
public:
    Rs485Controller();

    bool initializePort(std::string port, int bitrate);
    bool closePort();
    bool isOpen() {return m_fd > 0;};

    bool readData(std::vector<uint8_t> &out, int timeout = RS485_DEFAULT_READ_TIMEOUT);
    bool writeData(std::vector<uint8_t> data);
private:
    int m_fd = -1;
};


#endif // RS485CONTROLLER_H
