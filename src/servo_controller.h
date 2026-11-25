#ifndef SERVOCONTROLLER_H
#define SERVOCONTROLLER_H

#include "servo_struct.h"
#include "servo_def.h"

#include <string>
#include <memory>

namespace KtechServo {

class ServoController
{
public:
    ServoController();

    virtual bool initialize(int bitrate) = 0;
    virtual bool isInit() = 0;
    virtual bool motorOn(int id) = 0;
    virtual bool motorOff(int id) = 0;
    virtual bool motorStop(int id) = 0;
    virtual FullServoStatus readAllState(int id) = 0;
    virtual FullServoStatus readState1(int id) = 0;
    virtual FullServoStatus readState2(int id) = 0;
    virtual FullServoStatus readState3(int id) = 0;
    virtual bool writePid(int id, PIDData pid, WRITE_TYPE type) = 0;
    virtual PIDData readPid(int id) = 0;
    virtual bool clearError(int id) = 0;
    virtual ServoStatus multiLoopAngleCtl(int id, float angle, int speed) = 0;
    virtual ServoStatus singleLoopAngleCtl(int id, float angle, int speed) = 0;
    virtual ServoStatus incAngle(int id, float angle, int speed) = 0;
    virtual bool readAccel(int id, int32_t &accel) = 0;
    virtual bool writeAccel(int id, int32_t accel) = 0;
    virtual ServoStatus readMultiAngle(int id) = 0;
    virtual ServoStatus readSingleAngle(int id) = 0;
    virtual bool clearMotorAngle(int id) = 0;
    virtual EncoderData readEncoderData(int id) = 0;
    virtual bool writeEncoderData(int id, EncoderData data) = 0;
    virtual bool resetPos(int id) = 0;
    virtual ServoStatus setSpeed(int id, int speed) = 0;
    virtual ServoStatus setTorque(int id, int torque_value)  = 0;


};

}

#endif // SERVOCONTROLLER_H
