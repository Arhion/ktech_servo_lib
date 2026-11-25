#include "libservo_controller.h"
#include "can_controller.h"
#include "servo_can_controller.h"
#include "servo_rs485_controller.h"

std::unique_ptr<KtechServo::ServoController> servoController;
/*! \addtogroup extern servo controller API
    @{
*/

extern "C++" {

ServoCtl::ServoCtl()
{

}

/**
 * @brief initialize - initializes the class object
 * @param ifname - name of the interface, e.g., "can0"
 * @param bitrate - interface baud rate (for RS485 only; the bitrate for CAN is set via a separate, dedicated function)
 * @param type - type of control interface. KtechServo::CONTROL_CAN = 1, KtechServo::CONTROL_RS485 = 2
 * @return bool
 */
bool ServoCtl::initialize(std::string ifname, int bitrate, KtechServo::CONTROL_TYPE type)
{
    if(type == KtechServo::CONTROL_TYPE::CONTROL_CAN){
        servoController = std::make_unique<KtechServo::CanCtl::ServoCanController>(ifname);
        return servoController->initialize(bitrate);
    }else {
        servoController = std::make_unique<KtechServo::Rs485Ctl::ServoRs485Controller>(ifname);
        return servoController->initialize(bitrate);
    }

    return false;
}

/**
 * @brief deinitialize - deinitialization of a class object and closing of a connection
 * @return
 */
bool ServoCtl::deinitialize()
{
    if(servoController != nullptr){
        servoController.reset();
    }

    return true;
}

/**
 * @brief stopCan - Stops the CAN interface in the system. Requires root privileges.
 * @param ifname - name of the interface, e.g., "can0"
 * @return
 */
bool ServoCtl::startCan(std::string ifname)
{
#ifdef __linux__
    return can_do_start(ifname.c_str()) == 0;
#else
    return true;
#endif
}

/**
 * @brief stopCan - Stops the CAN interface in the system. Requires superuser privileges.
 * @param ifname - name of the interface, e.g., "can0"
 * @return
 */
bool ServoCtl::stopCan(std::string ifname)
{
#ifdef __linux__
    return can_do_stop(ifname.c_str()) == 0;
#else
    return true;
#endif
}

/**
 * @brief setCanBitrate - Sets the interface bitrate for a CAN bus. Requires superuser privileges.
 * @param ifname - name of the interface, e.g., "can0"
 * @param bitrate - operating bitrate
 * @return
 */
bool ServoCtl::setCanBitrate(std::string ifname, int bitrate)
{
#ifdef __linux__
   return  can_set_bitrate(ifname.c_str(), bitrate) == 0;
#else
   return true;
#endif
}

/**
 * @brief ServoCtl::isInit - checks whether the CAN interface has been initialized
 * @return
 */
bool ServoCtl::isInit()
{
    if(servoController != nullptr){
        return servoController->isInit();
    }

    return false;
}

/*
 * The servo commands group will only function after initialization.
 */

// servo commands
/**
 * @brief motorOn - starts the motor
 * @param id - servo device ID (1 - 32)
 * @return
 */
bool ServoCtl::motorOn(int id)
{
    if(servoController != nullptr){
        return servoController->motorOn(id);
    }

    return false;
}

/**
 * @brief motorOff - Turns off the motor
 * @param id - Servo device ID (1 - 32)
 * @return
 */
bool ServoCtl::motorOff(int id)
{
    if(servoController != nullptr){
        return servoController->motorOff(id);
    }

    return false;
}

/**
 * @brief motorStop - Stop motor 
 * @param id - Servo device ID (1 - 32)
 * @return
 */
bool ServoCtl::motorStop(int id)
{
    if(servoController != nullptr){
        return servoController->motorStop(id);
    }

    return false;
}

/**
 * @brief readAllState - Reads all servo states (state 1, state 2, state 3)
 * @param id - Servo device ID (1–32)
 * @return Returns a structure containing the servo states
 */
KtechServo::FullServoStatus ServoCtl::readAllState(int id)
{
    KtechServo::FullServoStatus out;

    if(servoController != nullptr){
        return servoController->readAllState(id);
    }

    return out;
}

/**
 * @brief readState1 - Reads the servo's status (temperature, voltage, errors)
 * @param id - Servo device ID (1–32)
 * @return Returns a structure containing the servo's status
 */
KtechServo::FullServoStatus ServoCtl::readState1(int id)
{
    KtechServo::FullServoStatus out;

    if(servoController != nullptr){
        return servoController->readState1(id);
    }

    return out;
}

/**
 * @brief readState2 - Reads the servo's status (temperature, torque, speed, encoder position)
 * @param id - Servo device ID (1–32)
 * @return Returns a structure containing the servo's status
 */
KtechServo::FullServoStatus ServoCtl::readState2(int id)
{
    KtechServo::FullServoStatus out;

    if(servoController != nullptr){
        return servoController->readState2(id);
    }

    return out;
}

/**
 * @brief readState3 - Reads the servo drive states (temperature, phase currents: A, B, C)
 * @param id - Servo drive device ID (1–32)
 * @return Returns a structure containing the servo drive states
 */
KtechServo::FullServoStatus ServoCtl::readState3(int id)
{
    KtechServo::FullServoStatus out;

    if(servoController != nullptr){
        return servoController->readState3(id);
    }

    return out;
}

/**
 * @brief writePid - Writes PID controller settings
 * @param id - Servo device ID (1 - 32)
 * @param pid - Structure containing the controller settings
 * @param type - Write type: WRITE_TYPE::WRITE_ROM for writing to ROM, WRITE_TYPE::WRITE_RAM for writing to RAM
 * @return
 */
bool ServoCtl::writePid(int id, KtechServo::PIDData pid, KtechServo::WRITE_TYPE type)
{
    if(servoController != nullptr){
        return servoController->writePid(id, pid, type);
    }

    return false;
}

/**
 * @brief readPid - Reads the PID controller settings
 * @param id - Servo device ID (1 - 32)
 * @return Returns a structure containing the PID controller settings
 */
KtechServo::PIDData ServoCtl::readPid(int id)
{
    KtechServo::PIDData out;
    if(servoController != nullptr){
        return servoController->readPid(id);
    }

    return out;
}

/**
 * @brief clearError - clears the list of errors
 * @param id - Servo device ID (1 - 32)
 * @return
 */
bool ServoCtl::clearError(int id)
{
    if(servoController != nullptr){
        return servoController->clearError(id);
    }

    return false;
}

/**
 * @brief multiLoopAngleCtl - Multi-loop angle control command (1/2) - Command for controlling motor position
 * @param id - Servo device ID (1–32)
 * @param angle - Angle in degrees between the current and desired position. The direction of movement is determined by the sign of the value. 
 * @param speed - Rotation speed, defined in units of 0.1 dps/LSB (degrees per second per LSB). For example, a value of 3600 corresponds to a rotation of 360 degrees in 1 second. 
 * @return Returns a simple structure containing the device status.
 */
KtechServo::ServoStatus ServoCtl::multiLoopAngleCtl(int id, float angle, int speed)
{
    KtechServo::ServoStatus out;
    if(servoController != nullptr){
        return servoController->multiLoopAngleCtl(id, angle, speed);
    }

    return out;
}

/**
 * @brief singleLoopAngleCtl - Single-loop angle control (1/2) - Command for controlling motor position
 * @param id - Servo device ID (1–32)
 * @param angle - Angle in degrees representing the displacement from the current position to the target position. The direction of movement is determined by the difference between the current position and the target position.
 * @param speed - Rotation speed, defined in units of 0.1 dps/LSB (degrees per second per LSB). For example, a speed of 3600 corresponds to a full 360-degree rotation in 1 second. 
 * @return Returns a simple structure containing the device status.
 */
KtechServo::ServoStatus ServoCtl::singleLoopAngleCtl(int id, float angle, int speed)
{
    KtechServo::ServoStatus out;
    if(servoController != nullptr){
        return servoController->singleLoopAngleCtl(id, angle, speed);
    }

    return out;
}

/**
 * @brief incAngle - Command to rotate by a specified angle (1/2)
 * @param id - Servo device ID (1–32)
 * @param angle - Angle in degrees between the current and desired position. The direction of movement is determined by the sign of the value. 
 * @param speed - Rotation speed, defined as 0.1 dps/LSB (e.g., a value of 3600 corresponds to a rotation of 360 degrees in 1 second). 
 * @return Returns a simple structure containing the device status.
 */
KtechServo::ServoStatus ServoCtl::incAngle(int id, float angle, int speed)
{
    KtechServo::ServoStatus out;
    if(servoController != nullptr){
        return servoController->incAngle(id, angle, speed);
    }

    return out;
}

/**
 * @brief readAccel - Read acceleration parameters
 * @param id - Servo device ID (1–32)
 * @param accel (out) - Output parameter value (in units of 1 dps/s, per documentation)
 * @return
 */
bool ServoCtl::readAccel(int id, int32_t &accel)
{
    if(servoController != nullptr){
        return servoController->readAccel(id, accel);
    }

    return false;
}

/**
 * @brief writeAccel - Command to write the acceleration parameter to the motor's RAM
 * @param id - Servo device ID (1–32)
 * @param accel (in) - Acceleration parameter (in units of 1 dps/s, per documentation)
 * @return
 */
bool ServoCtl::writeAccel(int id, int32_t accel)
{
    if(servoController != nullptr){
        return servoController->writeAccel(id, accel);
    }

    return false;
}

/**
 * @brief readMultiAngle - Reads the current multi-turn absolute position.
 * @param id - Servo device ID (1–32)
 * @return Returns a KtechServo::ServoStatus structure.
 */
KtechServo::ServoStatus ServoCtl::readMultiAngle(int id)
{
    KtechServo::ServoStatus out;
    if(servoController != nullptr){
        return servoController->readMultiAngle(id);
    }

    return out;
}

/**
 * @brief readSingleAngle - Command to read a single angle - Retrieves the current angular value
 * @param id - Servo device ID (1 - 32)
 * @return Returns a KtechServo::ServoStatus structure
 */
KtechServo::ServoStatus ServoCtl::readSingleAngle(int id)
{
    KtechServo::ServoStatus out;
    if(servoController != nullptr){
        return servoController->readSingleAngle(id);
    }

    return out;
}

/**
 * @brief clearMotorAngle - Command to reset the motor angle; sets the current position as the zero point. Does not function when power is off.
 * @param id - Servo device ID (1–32)
 * @return
 */
bool ServoCtl::clearMotorAngle(int id)
{
    if(servoController != nullptr){
        return servoController->clearMotorAngle(id);
    }

    return false;
}

/**
 * @brief readEncoderData - Retrieves encoder data
 * @param id - Servo device ID (1 - 32)
 * @return Returns a structure containing encoder parameters
 */
KtechServo::EncoderData ServoCtl::readEncoderData(int id)
{
    KtechServo::EncoderData out;
    if(servoController != nullptr){
        return servoController->readEncoderData(id);
    }

    return out;
}

/**
 * @brief writeEncoderData - Sets encoder parameters
 * @param id - Servo device ID (1 - 32)
 * @param data - Structure containing encoder parameters
 * @return
 */
bool ServoCtl::writeEncoderData(int id, KtechServo::EncoderData data)
{
    if(servoController != nullptr){
        return servoController->writeEncoderData(id, data);
    }

    return false;
}

/**
* @brief resetPos - writes the current encoder position to ROM memory
* @param id - servo device ID (1–32)
* @return
*/
bool ServoCtl::resetPos(int id)
{
    if(servoController != nullptr){
        return servoController->resetPos(id);
    }

    return false;
}

/**
 * @brief setSpeed ​​- sets the rotational speed. Applicable to the motor's current rotation.
 * @param id - Servo device ID (1–32)
 * @param speed - Rotational speed. Defined as 0.1 dps/LSB (degrees per second per least significant bit). For example, a value of 3600 corresponds to a 360-degree rotation in 1 second.
 * @return
 */
KtechServo::ServoStatus ServoCtl::setSpeed(int id, int speed)
{
    KtechServo::ServoStatus out;
    if(servoController != nullptr){
        return servoController->setSpeed(id, speed);
    }

    return out;
}

/**
 * @brief setTorque - Torque closed-loop control command - sets the torque value. `torque_value` is of type `int16_t`, with a range of -2048 to 2048.
 * @param id - Servo device ID (1–32)
 * @param torque_value
 * @return Returns a simple structure containing the device status
 */
KtechServo::ServoStatus ServoCtl::setTorque(int id, int torque_value)
{
    KtechServo::ServoStatus out;
    if(servoController != nullptr){
        return servoController->setTorque(id, torque_value);
    }

    return out;
}
}
/*! @} */
