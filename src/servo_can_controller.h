#ifndef SERVOCANCONTROLLER_H
#define SERVOCANCONTROLLER_H

#include "servo_controller.h"
#include "can_controller.h"
#include "servo_can_struct.h"
#include "utils.h"
#include "servo_def.h"

#include <memory>

namespace KtechServo {
namespace CanCtl {

class ServoCanController : public ServoController
{
public:
    ServoCanController(){};
    ServoCanController(std::string can_iface);

    bool initialize(int bitrate);
    bool isInit();
    bool setBitrate();
    bool deinitialize();
    bool startCan();
    bool stopCan();

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
    ServoStatus multiLoopAngleCtl(int id, float angle, int speed = 0);
    ServoStatus singleLoopAngleCtl(int id, float angle, int speed = 0);
    ServoStatus incAngle(int id, float angle, int speed = 0);
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
    bool sendData(int id, CanData data);
    CanData readResp(int id, int timeout = -1);

    bool sendAndCheckCommonResp(int id, int com);
    bool sendAndCheckDataResp(int id, int com, CanData &data);

    // parse
    int parseCommonResp(CanData data);
    StatusResp_t parseStatusMsg(CanData data);
    PIDRespData parsePidMsg(CanData data);
    ReadAccelRespData parseAccelMsg(CanData data);
    EncRespData parseEncDataMsg(CanData data);
    EncZeroPointRespData parseEncOffsetDataMsg(CanData data);
    MotorStateAndErrorRespData parseStateRespMsg(CanData data);
    MotorStateRespData2 parseStateRespMsg2(CanData data);
    MotorStateRespData3 parseStateRespMsg3(CanData data);
    MALComData parseMultiAngleRespMsg(CanData data);
    SALComData parseSingleAngleRespMsg(CanData data);

private:
    std::unique_ptr<CanController> m_can_ctl;

};

}
}

#endif // SERVOCANCONTROLLER_H
