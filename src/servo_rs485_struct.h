#ifndef SERVO_RS485_STRUCT_H
#define SERVO_RS485_STRUCT_H

#include <cstdint>
#include <stdint.h>

namespace KtechServo{
namespace Rs485Ctl {

#pragma pack(push, 1) // выравнивание по одному байту

// common command

struct CommandFrame
{
    uint8_t frame_head = 0x3E;
    uint8_t command;
    uint8_t id;
    uint8_t data_lenght = 0;
    uint8_t checksum;
};

// 1. Read motor state 1 and error state command
// com as common com

struct MotorStateAndErrorRespData
{
    char temp;
    uint16_t voltage;
    uint8_t null_data2[2] = {0,0};
    uint8_t motor_state = 0;
    uint8_t error_state;
    uint8_t data_checksum;
};


// 2. Clear error command
// com as common com
// resp as MotorStateAndErrorRespData


// 3. Read motor state 2 command
// com as common com

struct MotorStateRespData2
{
    char temp;
    uint16_t torque;
    int16_t speed;
    uint16_t encoder_pos;
    uint8_t data_checksum;
};

// 4. Read motor state 3 command
// com as common com

struct MotorStateRespData3
{
    char temp;
    int16_t phase_a;
    int16_t phase_b;
    int16_t phase_c;
    uint8_t data_checksum;
};

// 5. Motor off command
// com as common com
// resp as common com

// 6. Motor on command
// com as common com
// resp as common com

// 7. Motor stop command
// com as common com
// resp as common com

// 7. Motor stop command
// com as common com
// resp as common com

// 8. Control and read holding brake state command
// com data

struct HoldingBrakeCtlComData
{
    uint8_t holdingState;
    uint8_t data_checksum;
};

typedef HoldingBrakeCtlComData HoldingBrakeRespData;

// 9. Open loop control command (ms only)

struct OlCtlComData
{
    uint16_t open_loop_value;
    uint8_t data_checksum;
};
// resp as MotorStateRespData2

// 10. Torque closed loop control command(the command can only be applied to MF,MH,MG series)

struct TorqueClCtlComData
{
    uint16_t torque_value;
    uint8_t data_checksum;
};
// resp as MotorStateRespData2


// 11. Speed closed loop control command

struct SpeedClCtlComData
{
    uint32_t speed_value;
    uint8_t data_checksum;
};
// resp as MotorStateRespData2

// 12. Multi loop angle control command 1

struct MlAngleCtlComData1
{
    int64_t angle_value;
    uint8_t data_checksum;
};
// resp as MotorStateRespData2

// 13. Multi loop angle control command 2

struct MlAngleCtlComData2
{
    int64_t angle_value;
    uint32_t speed;
    uint8_t data_checksum;
};
// resp as MotorStateRespData2

// 14. Single loop angle control 1

struct SlAngleCtlComData1
{
    uint8_t spin_dir;
    uint16_t angle_value;
    uint8_t null_data = 0;
    uint8_t data_checksum;
};
// resp as MotorStateRespData2

// 15. Single loop angle control 2

struct SlAngleCtlComData2
{
    uint8_t spin_dir;
    uint16_t angle_value;
    uint8_t null_data = 0;
    uint32_t speed;
    uint8_t data_checksum;
};
// resp as MotorStateRespData2

// 16. Increment angle control command1

struct IncAngleCtlComData1
{
    int32_t angle_value;
    uint8_t data_checksum;
};
// resp as MotorStateRespData2

// 17. Increment angle control command 2

struct IncAngleCtlComData2
{
    int32_t angle_value;
    uint32_t speed;
    uint8_t data_checksum;
};
// resp as MotorStateRespData2

// 18. Read encoder command
//com as common com

struct EncRespData
{
    uint16_t encoder_data;
    uint16_t encoder_raw_pos;
    uint16_t encoder_offset;
    uint8_t data_checksum;
};

// 19. Write current position to ROM as the motor zero point command
// com as common command

struct EncZeroPointRespData
{
    uint16_t encoder_data;
    uint8_t data_checksum;
};

// 20. Read multi angle loop command
// com as common command

struct MALRespData
{
    int64_t angle;
    uint8_t data_checksum;
};

// 21. Clear multi-loop angle command
// com as common com
// resp as common com

// 22. Read single angle loop command
// com as common command

struct SALRespData
{
    uint32_t angle;
    uint8_t data_checksum;
};

// 23. Write the current position to RAM as the motor zero command
// com as common com
// resp as common com

// 24. Read PID parameter command
// com as Common_command

struct PIDComData
{
    uint8_t parameter_id;
    uint8_t null_data = 0;
    uint8_t data_checksum;
};

struct PIDRespData
{
    uint8_t parameter_id;
    //    uint8_t parameter[6] = {0,0,0,0,0,0};
    //    uint8_t data_checksum;
};

// parparameters:
// angle pid id = 150 (0x96)
struct AnglePid
{
    uint16_t anngle_pid_kp;
    uint16_t anngle_pid_ki;
    uint16_t anngle_pid_kd;
};

// speed pid id = 151 (0x97)
struct SpeedPid
{
    uint16_t speed_pid_kp;
    uint16_t speed_pid_ki;
    uint16_t speed_pid_kd;
};

// current pid id = 152 (0x98)
struct CurrentPid
{
    uint16_t current_pid_kp;
    uint16_t current_pid_ki;
    uint16_t current_pid_kd;
};

// max torque pid id = 153 (0x99)
struct TorqueParam
{
    int16_t torque_current;
};

// max speed id = 154 (0x9A)
struct MaxSpeedParam
{
    int32_t max_speed;
};

// max angle low bytes id = 155 (0x9B)
struct MaxAngleParamLow
{
    uint32_t max_angle;
};

// max angle high bytes id = 156 (0x9C)
struct MaxAngleParamHigh
{
    uint32_t max_angle;
};

// current ramp id = 157 (0x9D)
struct CurrentRampParam
{
    int16_t current_ramp;
};

// speed ramp id = 158 (0x9E)
struct SpeedRampParam
{
    int32_t speed_ramp;
};

// 25. Write PID parameter to RAM command
typedef  PIDRespData WritePIDComData;

struct WritePidRespData
{
    uint8_t parameter_id;
    uint8_t write_status;
    uint8_t data_checksum;
};

// 26. Write PID parameter to RAM command
// com as PIDComData
// resp WritePidRespData

// 27. Read driver and motor type command
// com as common com

struct MotorInfoRespData
{
    uint8_t driverName[20];    // Driver name
    uint8_t motorName[20];     // Motor name
    uint8_t motorId[12];       // Motor id
    uint16_t hardwareVersion;  // Driver hardware version
    uint16_t motorVersion;     // Motor version
    uint16_t firmwareVersion;  // Firmware version

    uint8_t data_checksum;

};


#pragma pack(pop)
}
}



#endif // SERVO_RS485_STRUCT_H
