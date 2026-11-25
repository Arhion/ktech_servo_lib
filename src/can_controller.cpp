#include "can_controller.h"

CanController::CanController(std::string can_name)
{
    m_can_name = can_name;
}

bool CanController::openIface()
{
#ifdef __linux__
    if ((m_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Socket");
        return false;
    }
#endif

    return true;
}

bool CanController::closeIface()
{
#ifdef __linux__
    if (close(m_fd) < 0) {
        perror("Close");
        return false;
    }

#endif
    return true;
}

bool CanController::isOpen()
{
    return m_fd > 0;
}

bool CanController::stopCan()
{
    bool status = true;
#ifdef __linux__
    const char* ifname = m_can_name.c_str();        // Name CAN
    status = can_do_stop(ifname) == 0;
#endif

    return status;
}

bool CanController::startCan()
{
    bool status = true;
#ifdef __linux__
    const char* ifname = m_can_name.c_str();        // Name CAN
    status = can_do_start(ifname) == 0;
#endif

    return status;
}

bool CanController::setBitRate(int speed)
{
    bool status = true;

    if(m_can_name.size() <= 0){
        return status;
    }
    const char* ifname = m_can_name.c_str();        // Name CAN
    unsigned int bitrate = speed;

#ifdef __linux__

    if(!this->stopCan()){
        return false;
    }

    status = can_set_bitrate(ifname, bitrate) == 0;

    if(!status){
        return false;
    }

    if(!this->startCan()){
        return false;
    }
#endif

    return status;
}

bool CanController::writeData(CanData can_data)
{
    if(can_data.data.size() <= 0){
        return false;
    }

    if(m_can_name.size() <= 0){
        return false;
    }

#ifdef __linux__
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;

#ifdef _DEBUG_
    printf("CAN Sockets send\r\n");
#endif


    strcpy(ifr.ifr_name, m_can_name.c_str());
    ioctl(m_fd, SIOCGIFINDEX, &ifr);

    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(m_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Bind");
        return false;
    }

    frame.can_id = can_data.id;
    frame.can_dlc = can_data.data.size();
    memcpy(frame.data, (char*)can_data.data.data(), frame.can_dlc);

    if (write(m_fd, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("Write");
        return false;
    }

#endif
    std::this_thread::sleep_for(std::chrono::microseconds(10));

    return true;
}

CanData CanController::readData(int id, int timeout)
{
    CanData can_data;

    if(m_can_name.size() <= 0){
        return can_data;
    }

#ifdef __linux__

    int nbytes;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;

#ifdef _DEBUG_
    printf("CAN Sockets Receive\r\n");
#endif

    strcpy(ifr.ifr_name, m_can_name.c_str());
    ioctl(m_fd, SIOCGIFINDEX, &ifr);

    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(m_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Bind");
        return can_data;
    }

    struct timeval tv;
    fd_set  rset;
    int ret = 0;

    tv.tv_sec = timeout;
    tv.tv_usec = 0;

    FD_ZERO(&rset);
    FD_SET(m_fd, &rset);

    if ((ret = select(m_fd + 1, &rset, NULL, NULL, &tv)) < 0){
        // select
    } else if(ret == 0){
        puts("timed out");
    }

    if (FD_ISSET(m_fd, &rset)) {
        nbytes = read(m_fd, &frame, sizeof(struct can_frame));
    } else {
        return can_data;
    }

    if(nbytes < 0) {
        perror("Read");
        return can_data;
    }

#ifdef _DEBUG_
    printf("0x%03X [%d] ",frame.can_id, frame.can_dlc);
#endif

    for (int i = 0; i < frame.can_dlc; i++){
#ifdef _DEBUG_
        printf("%02X ",frame.data[i]);
#endif
        can_data.data.push_back(frame.data[i]);
    }
#ifdef _DEBUG_
    printf("\r\n");
#endif


    can_data.status = true;
#endif

    return can_data;
}
