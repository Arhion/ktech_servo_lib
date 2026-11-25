#include "rs485_controller.h"

Rs485Controller::Rs485Controller()
{

}

bool Rs485Controller::initializePort(std::string port, int bitrate_in)
{
    if(this->isOpen()){
        return false;
    }

#ifdef __linux__
    m_fd = open(port.c_str(), O_RDWR);

    if (m_fd < 0) {
        return false;
    }

    int bitrate = -1;
    switch(bitrate_in)
    {
    case Rs485Bitrate::RS485_9600:
        bitrate = B9600;
        break;
    case Rs485Bitrate::RS485_19200:
        bitrate = B19200;
        break;
    case Rs485Bitrate::RS485_38400:
        bitrate = B38400;
        break;
    case Rs485Bitrate::RS485_57600:
        bitrate = B57600;
        break;
    case Rs485Bitrate::RS485_115200:
        bitrate = B115200;
        break;
    case Rs485Bitrate::RS485_230400:
        bitrate = B230400;
        break;
    case Rs485Bitrate::RS485_460800:
        bitrate = B460800;
        break;
    case Rs485Bitrate::RS485_1000000:
        bitrate = B1000000;
        break;
    case Rs485Bitrate::RS485_2000000:
        bitrate = B2000000;
        break;
    default:
        close(m_fd);
        return false;
    }

    struct termios options;

    tcgetattr(m_fd, &options);
    cfmakeraw(&options);
    cfsetispeed(&options, bitrate);
    cfsetospeed(&options, bitrate);

    tcflush(m_fd, TCIFLUSH);
    tcsetattr(m_fd, TCSANOW, &options);

#endif

    return true;
}

bool Rs485Controller::closePort()
{
    if(!this->isOpen()){
        return false;
    }

    bool status = false;
#ifdef __linux__
    status = close(m_fd) == 0;
#endif
    return status;
}

bool Rs485Controller::readData(std::vector<uint8_t> &out, int timeout)
{
    if(!this->isOpen()){
        return false;
    }

    out.clear();

    int nbytes = 0;
#ifdef __linux__

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
        ioctl(m_fd, FIONREAD, &nbytes);
        if(nbytes > 0){
            out.resize(nbytes);
            nbytes = read(m_fd, out.data(), nbytes);
        }
    } else {
        return false;
    }

    if(nbytes < 0) {
        perror("Read");
        return false;
    }
#endif

    return nbytes > 0;
}

bool Rs485Controller::writeData(std::vector<uint8_t> data)
{
    if(!this->isOpen()){
        return false;
    }
    if(data.size() <= 0){
        return false;
    }

    int nbytes = 0;

#ifdef __linux__

    nbytes = write(m_fd, (char*)data.data(), sizeof(char) * data.size());

    if(nbytes <= 0) {
        return false;
    }
#endif

    return nbytes == (int)data.size();
}
