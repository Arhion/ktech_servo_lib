#ifndef SERVO_CAN_STRUCT_H
#define SERVO_CAN_STRUCT_H

#include <cstdint>

namespace KtechServo{
namespace CanCtl {

struct TorqueClCtlComData; // forward declaration

typedef  TorqueClCtlComData StatusResp_t;

#pragma pack(push, 1) 
struct CommonCommandData
{
   uint8_t com;
   uint8_t null_data[7] = {0,0,0,0,0,0,0};
};

typedef  CommonCommandData CommonRespData_t;

// 4. Open loop control command (ms only)
struct OlCtlComData
{
   uint8_t com;
   uint8_t null_data1[3] = {0,0,0};
   uint16_t power_ctl;
   uint8_t null_data2[2] = {0,0};
};

struct OlCtlRespData
{
   uint8_t com;
   uint8_t motor_temp;
   uint16_t torque_current;
   uint16_t speed;
   uint16_t encoder_pos;
};

// 5. Torque closed loop control command(the command can only be applied to MF,MH,MG series)
struct TorqueClCtlComData
{
   uint8_t com;
   uint8_t null_data1[3] = {0,0,0};
   uint16_t torque_value;
   uint8_t null_data2[2] = {0,0};
};

struct TorqueClCtlRespData
{
   uint8_t com;
   uint8_t motor_temp;
   uint16_t torque_current;
   uint16_t speed;
   uint16_t encoder_pos;
};
// Torque_closed_loop_ctl_resp: 5 6 7 8 9 10 11 12

// 6. Speed closed loop control command
struct SpeedClCtlComData
{
   uint8_t com;
   uint8_t null_data1[3] = {0,0,0};
   uint32_t speed_value;
};
// resp as Torque_closed_loop_ctl_resp

// 7. Multi loop angle control command 1
struct MlAngleCtlComData1
{
   uint8_t com;
   uint8_t null_data1[3] = {0,0,0};
   int32_t angle_value;
};
// resp as Torque_closed_loop_ctl_resp

// 8. Multi loop angle control command 2
struct MlAngleCtlComData2
{
   uint8_t com;
   uint8_t null_data = 0;
   uint16_t speed;
   uint32_t angle_value;
};
// resp as Torque_closed_loop_ctl_resp

/// 9. Single loop angle control 1
struct SlAngleCtlComData1
{
   uint8_t com;
   uint8_t spin_dir;
   uint8_t null_data1[2] = {0,0};
   uint32_t angle_value;
};
// resp as Torque_closed_loop_ctl_resp

// 10. Single loop angle control 2
struct SlAngleCtlComData2
{
   uint8_t com;
   uint8_t spin_dir;
   uint16_t speed;
   uint32_t angle_value;
};
// resp as Torque_closed_loop_ctl_resp

// 11. Increment angle control command1
struct IncAngleCtlComData1
{
   uint8_t com;
    uint8_t null_data1[3] = {0,0,0};
	uint32_t angle_value;

};
// resp as Torque_closed_loop_ctl_resp


// 12. Increment angle control command 2
struct IncAngleCtlComData2
{
    uint8_t com;
    uint8_t null_data1 = 0;
    uint16_t speed;
    int32_t angle_value;
};
// resp as Torque_closed_loop_ctl_resp

// 13. Read PID parameter command
// com as Common_command

struct PIDRespData
{
    uint8_t com;
    uint8_t null_data = 0;
    uint8_t pos_p_value;
    uint8_t pos_i_value;
    uint8_t speed_p_value;
    uint8_t speed_i_value;
    uint8_t torque_p_value;
    uint8_t torque_i_value;
};

// 14. Write PID parameters to RAM command
// 15. Write PID parameters to ROM command
typedef  PIDRespData PIDComData;

// resp as write pid com

// 16. Read acceleration command
// com as common command

struct ReadAccelRespData
{
    uint8_t com;
    uint8_t null_data[3] = {0,0,0};
    uint32_t accel_data;
};

// 17. Write acceleration to RAM command
// com as ReadAccelResp
typedef  ReadAccelRespData WriteAccelComData;

// 18. Read encoder command
//com as common com

struct EncRespData
{
    uint8_t com;
    uint8_t null_data = 0;
    uint16_t encoder_data;
    uint16_t encoder_raw_pos;
    uint16_t encoder_offset;
};

// 19. Write encoder value to ROM as the motor zero point command(

struct EncZeroPointComData
{
    uint8_t com;
    uint8_t null_data[5] = {0,0,0,0,0};
    uint16_t encoder_offset;
};

// 20. Write current position to ROM as the motor zero point command
// com as common command
typedef  EncZeroPointComData EncZeroPointRespData;

// 21. Read multi angle loop command
// com as common command

struct MALComData
{
    uint8_t com;
    int64_t angle;
};


// 22. Read single angle loop command
// com as common command

struct SALComData
{
    uint8_t com;
    uint8_t null_data[3] = {0,0,0};
    uint32_t angle;
};

// 23. Clear motor angle loop command
// com as common com

// 24. Read motor state 1 and error state command
// com as common com

struct MotorStateAndErrorRespData
{
   uint8_t com;
   char temp;
   uint8_t null_data1 = 0;
   uint16_t voltage;
   uint8_t null_data2[2] = {0,0};
   uint8_t error_state;
};

// 25. Clear motor error state
// com as common com
// resp as MotorStateAndErrorRespData

// 26. Read motor state 2 command
// com as common com

struct MotorStateRespData2
{
    uint8_t com;
    char temp;
    uint16_t torque;
    int16_t speed;
    uint16_t encoder_pos;
};

// 27. Read motor state 3 command
// com as common com
struct MotorStateRespData3
{
    uint8_t com;
    char temp;
    int16_t phase_a;
    int16_t phase_b;
    int16_t phase_c;
};

#pragma pack(pop)

}
}

#endif // SERVO_CAN_STRUCT_H
