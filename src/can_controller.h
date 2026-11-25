#ifndef CANCONTROLLER_H
#define CANCONTROLLER_H

#include <iostream>
#include <cstring>
#include <vector>
#include <thread>
#ifdef __linux__
#include <libsocketcan.h>

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#endif

#include "servo_def.h"

#define CAN_DEFAULT_READ_TIMEOUT 5 //s

struct CanData
{
    int id = 0;
    bool status = false;
    std::vector<uint8_t> data;

public:
    void clear(){
        id = 0;
        status = false;
        data.clear();
    }
};


class CanController
{
public:
    CanController(std::string can_name);

    bool openIface();
    bool closeIface();
    bool isOpen();

    bool stopCan(); // Root privileges are required
    bool startCan(); // Root privileges are required
    bool setBitRate(int speed); // Changing the speed turns off the interface!!!! Root privileges are required

    bool writeData(CanData data);
    CanData readData(int id, int timeout = CAN_DEFAULT_READ_TIMEOUT);

private:
    std::string m_can_name = "";
    int m_fd = -1;


};

#endif // CANCONTROLLER_H
