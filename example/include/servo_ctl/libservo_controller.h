#ifndef LIBSERVO_CONTROLLER_H
#define LIBSERVO_CONTROLLER_H

#include "servo_def.h"
#include "servo_struct.h"
#include "servo_controller.h"

#include <string>
#include <memory>


//#define __MODULE_DEBUG //

#ifndef __MODULE_DEBUG
#if defined(_MSC_VER)
    //  Microsoft
#define EXPORT __declspec(dllexport)
#define IMPORT __declspec(dllimport)

#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>

#elif defined(__GNUC__)
    //  GCC
#define EXPORT __attribute__((visibility("default")))
#define IMPORT
#else
#define EXPORT
#define IMPORT
#endif

#endif // __MODULE_DEBUG

#ifdef __MODULE_DEBUG
#define EXPORT

#endif

extern "C++" {

class EXPORT ServoCtl
{
public:
    ServoCtl();

    bool initialize(std::string ifname, int bitrate = 115200, KtechServo::CONTROL_TYPE type = KtechServo::CONTROL_TYPE::CONTROL_CAN);
    bool deinitialize();
    bool startCan(std::string ifname);
    bool stopCan(std::string ifname);
    bool setCanBitrate(std::string ifname, int bitrate);
    bool isInit();

    // The servo commands group will only function after initialization

    bool motorOn(int id);
    bool motorOff(int id);
    bool motorStop(int id);
    KtechServo::FullServoStatus readAllState(int id);
    KtechServo::FullServoStatus readState1(int id);
    KtechServo::FullServoStatus readState2(int id);
    KtechServo::FullServoStatus readState3(int id);
    bool writePid(int id, KtechServo::PIDData pid, KtechServo::WRITE_TYPE type);
    KtechServo::PIDData readPid(int id);
    bool clearError(int id);
    KtechServo::ServoStatus multiLoopAngleCtl(int id, float angle, int speed = 0);
    KtechServo::ServoStatus singleLoopAngleCtl(int id, float angle, int speed = 0);
    KtechServo::ServoStatus incAngle(int id, float angle, int speed = 0);
    bool readAccel(int id, int32_t &accel);
    bool writeAccel(int id, int32_t accel);
    KtechServo::ServoStatus readMultiAngle(int id);
    KtechServo::ServoStatus readSingleAngle(int id);
    bool clearMotorAngle(int id);
    KtechServo::EncoderData readEncoderData(int id);
    bool writeEncoderData(int id, KtechServo::EncoderData data);
    bool resetPos(int id);
    KtechServo::ServoStatus setSpeed(int id, int speed);
    KtechServo::ServoStatus setTorque(int id, int torque_value);


private:
    std::unique_ptr<KtechServo::ServoController> servoController;
};

}


#endif // LIBSERVO_CONTROLLER_H
