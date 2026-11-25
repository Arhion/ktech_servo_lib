#ifndef SERVO_STRUCT_H
#define SERVO_STRUCT_H

#include <cstdint>
#include <stdio.h>

namespace KtechServo{

struct ServoStatus
{
    bool status = false;
    float angle = 0;
    uint8_t id = 0;
    uint8_t error = 0;
    uint16_t voltage = 0;
    char temp = 0;
    uint32_t speed = 0;
    uint8_t spin_dir = 0;
    uint16_t speed_limit = 0;
    int16_t toqrue_value = 0;
    int16_t power_control = 0;
    int32_t acceleration = 0;
    uint16_t encoder_pos = 0;
};

struct FullServoStatus
{
    bool status = false;
    float angle = 0;
    uint8_t id = 0;
    uint8_t error = 0;
    uint16_t voltage = 0;
    char temp = 0;
    int32_t speed = 0;
    uint8_t spin_dir = 0;
    uint16_t speed_limit = 0;
    int16_t torque_value = 0;
    int16_t power_control = 0;
    int32_t acceleration = 0;

    uint16_t encoder_offset = 0;
    uint16_t encoder_pos = 0;
    uint16_t encoder = 0;

    uint16_t phase_a = 0;
    uint16_t phase_b = 0;
    uint16_t phase_c = 0;

    uint8_t motor_state = 0;

};


struct EncoderData
{
    bool status = false;
    uint16_t encoder_offset = 0;
    uint16_t encoder_pos = 0;
    uint16_t encoder = 0;
};


struct PIDData
{
    // PID
    bool status = false;
    uint8_t pos_p_value = 0;
    uint8_t pos_i_value = 0;
    uint8_t speed_p_value = 0;
    uint8_t speed_i_value = 0;
    uint8_t torque_p_value = 0;
    uint8_t torque_i_value = 0;
};

}


#endif // SERVO_STRUCT_H
