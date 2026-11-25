#ifndef SERVORS485CONTROLLER_H
#define SERVORS485CONTROLLER_H

#include <cstring>

#include "servo_controller.h"
#include "servo_def.h"
#include "servo_struct.h"
#include "rs485_controller.h"
#include "servo_rs485_struct.h"

#include "utils.h"

#define RS485_COMMAND_FRAME_LEN 5

namespace KtechServo {
namespace Rs485Ctl {

// supported baudrates:
// 9600, 19200, 38400,57600,
// 115200(default),230400,460800,1Mbps,2Mbps

class ServoRs485Controller : public ServoController
{
public:
    ServoRs485Controller(){};
    ServoRs485Controller(std::string rs485_iface);

    bool initialize(int bitrate);
    bool isInit();
    bool deinitialize();

    // ServoController interface
public:
    bool motorOn(int id);
    bool motorOff(int id);
    bool motorStop(int id);
    FullServoStatus readAllState(int id);
    FullServoStatus readState1(int id);
    FullServoStatus readState2(int id);
    FullServoStatus readState3(int id);
    bool writePid(int id, PIDData pid, WRITE_TYPE type);
    PIDData readPid(int id);
    bool clearError(int id);
    ServoStatus multiLoopAngleCtl(int id, float angle, int speed);
    ServoStatus singleLoopAngleCtl(int id, float angle, int speed);
    ServoStatus incAngle(int id, float angle, int speed);
    bool readAccel(int id, int32_t &accel);
    bool writeAccel(int id, int32_t accel);
    ServoStatus readMultiAngle(int id);
    ServoStatus readSingleAngle(int id);
    bool clearMotorAngle(int id);
    EncoderData readEncoderData(int id);
    bool writeEncoderData(int id, EncoderData data);
    bool resetPos(int id);
    ServoStatus setSpeed(int id, int speed);
    ServoStatus setTorque(int id, int torque_value);

private:
    bool sendCommonCommand(int id, int com);
    bool sendData(int id, int com, std::vector<uint8_t> &data);
    void createCommandFrame(std::vector<uint8_t> &out, int id, int com, int len);
    std::vector<uint8_t> readResp(int timeout = RS485_DEFAULT_READ_TIMEOUT);
    bool sendAndCheckCommonResp(int id, int com);

    template<typename DataStruct, typename RespStruct>
    bool sendAndGetDataResp(int id, int com, const DataStruct &data_frame, RespStruct &resp);
    template<typename DataStruct, typename RespStruct>
    ServoStatus sendComAndGetServoState(int id, int com, const DataStruct &data_frame, RespStruct &resp);

    uint8_t calcChecksum(std::vector<uint8_t> &data, int size);
    uint8_t calcChecksum(uint8_t *data, int size);


    // parse
    template<typename RespStruct>
    bool parseFrame(RespStruct &out_data, int com, std::vector<uint8_t> &data);
    template<typename RespStruct>
    RespStruct parseDataResp(std::vector<uint8_t> &data, int point = 0);


private:
    std::unique_ptr<Rs485Controller> m_rs485_ctl;
    std::string m_rs485_iface = "";
};

}
}

#endif // SERVORS485CONTROLLER_H
