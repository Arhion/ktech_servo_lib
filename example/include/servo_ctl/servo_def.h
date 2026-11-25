#ifndef SERVO_DEF_H
#define SERVO_DEF_H
// http://en.lkmotor.cn/Download.aspx?ClassID=21

//#define _DEBUG_


// SHANGHAI LINGKONG TECHNOLOGY CO.,LTD
// CAN PROTOCOL V2.35
// #*. - chapter number from the manual

#define SERVO_CAN_MSG_LEN 8
#define STANDERT_ID_OFFSET 0x140
#define ANGLE_RATIO float(0.001)
#define M_ANGLE_RATIO float(0.001)

#define SPEED_CTL_RATIO float(0.01)

namespace KtechServo {

enum WRITE_TYPE
{
    WRITE_NONE = 0,
    WRITE_ROM,
    WRITE_RAM
};

enum class CONTROL_TYPE
{
    CONTROL_NONE = 0,
    CONTROL_CAN,
    CONTROL_RS485
};



namespace CanCtl {



enum CAN_SINGLE_MOTOR_COMMAND
{
    CAN_SMC_NONE = 0, // NONE
    CAN_MOTOR_OFF = 0x80, // #1. Switch the motor from the on state (the default state after power-on) to the off state,
    // clear motor turns and the earlier command.
    CAN_MOTOR_ON = 0x88, // #2. Swift the motor from off state to on state.
    CAN_MOTOR_STOP = 0x81, // #3. Stop motor but don’t clear the motor state.
    CAN_OPEN_LOOP = 0xA0, // #4. Host send commands to control the open loop voltage (only servo MS series)
    CAN_TORQUE_CLOSED_LOOP_CTL = 0xA1, // #5. Host send commands to control the torque current output (only MF, MH, MG series)
    CAN_SPEED_CLOSED_LOOP_CTL = 0xA2, // #6. Host send commands to control motor speed.
    CAN_MULTI_LOOP_ANGLE_CTL = 0xA3, // #7. Host send this command to control the position of the motor(Multi turn angle)
    CAN_MULTI_LOOP_ANGLE_CTL_2 = 0xA4, // #8. Host send this command to control the position of the motor(Multi turn angle).
    CAN_SINGLE_LOOP_ANGLE_CTL = 0xA5, // #9. Host send this command to control the position of the motor(Single turn angle)
    CAN_SINGLE_LOOP_ANGLE_CTL_2 = 0xA6, // #10. Host send this command to control the position of the motor(single turn angle).
    CAN_INC_ANGLE_CTL = 0xA7, // #11. Host send commands to control the increment angle of the motor.
    CAN_INC_ANGLE_CTL_2 = 0xA8, // #12. Host send commands to control the increment angle of the motor.
    CAN_READ_PID = 0x30, // #13. Host send this command to read current motor PID parameters.
    CAN_WRITE_PID_TO_RAM = 0x31, // #14. Host send this command to write PID parameters to RAM, parameter are invalid when power off.
    CAN_WRITE_PID_TO_ROM = 0x31, // #15. Host sent this command to write PID paramters to ROM, parameters are valid when power off.
    CAN_READ_ACCELERATION = 0x33, // #16. Host sent this command to write PID paramters to ROM, parameters are valid when power off.
    CAN_WRITE_ACCELETATION = 0x34, // #17. Host sent this command to read motor acceleration parameter.
    CAN_READ_ENCODER_VALUE = 0x90, // #18. Host send commands to write acceleration to RAM.Parameter are invalid when power off.
    CAN_WRITE_ENCODER_VALUE = 0x91, // #19. Host send commands to read current encoder position.
    CAN_WRITE_CURRENT_POS = 0x19, // #20. Host send this command to set encoder offset.
    CAN_READ_MULTI_ANGLE_LOOP = 0x92, // #21. Write motor encoder current position to ROM as the initial position.
    CAN_READ_SINGLE_ANGLE_LOOP = 0x94, // #22. Host send this command to read motor current single angle value.
    CAN_CLEAR_MOTOR_ANGLE_LOOP = 0x95, // #23. This command clear motor multi turn and single turn data and set
    // current position as motor zero point. It’s invalid when power off.
    CAN_READ_MOTOR_STATE = 0x9A, // #24. This command read current motor temperature, voltage and error state.
    CAN_CLEAR_MOTOR_ERROR_STATE = 0x9B, // #25. This command is to clear motor current error state.
    CAN_READ_MOTOR_STATE_2 = 0x9C, // #26. This command read current temperature,voltage,speed and encoder position.
    CAN_READ_MOTOR_STATE_3 = 0x9D, // #27. This command read current motor temperature and phase current data.
};

}

namespace Rs485Ctl {

// SHANGHAI LINGKONG TECHNOLOGY CO.,LTD
// RS485 PROTOCOL V2.35
// #*. - chapter number from the manual

enum RS485_SINGLE_MOTOR_COMMAND
{
    RS485_SMC_NONE = 0, // NONE
    RS485_READ_MOTOR_STATE = 0x9A, // #1. This command read current motor temperature, voltage and error state.
    RS485_CLEAR_MOTOR_ERROR_STATE = 0x9B, // #2. This command is to clear motor current error state.
    RS485_READ_MOTOR_STATE_2 = 0x9C, // #3. This command read current temperature, torque (MF, MG), output power (MS),speed and encoder position.
    RS485_READ_MOTOR_STATE_3 = 0x9D, // #4. This command read current motor temperature and phase current data.
    RS485_MOTOR_OFF = 0x80, // #5. Switch the motor from the on state (the default state after power-on) to the off state,
    // clear motor turns and the earlier command.
    RS485_MOTOR_ON = 0x88, // #6. Swift the motor from off state to on state.
    RS485_MOTOR_STOP = 0x81, // #7. Stop motor but don’t clear the motor state.
    RS485_CTL_AND_READ_HOLDING_BRAKE = 0x8C, // #8. Control holding brake or read current holding brake state.
    RS485_OPEN_LOOP = 0xA0, // #9. Host send commands to control the open loop voltage (only servo MS series)
    RS485_TORQUE_CLOSED_LOOP_CTL = 0xA1, // #10. Host send commands to control the torque current output (only MF, MH, MG series)
    RS485_SPEED_CLOSED_LOOP_CTL = 0xA2, // #11. Host send commands to control motor speed.
    RS485_MULTI_LOOP_ANGLE_CTL = 0xA3, // #12. Host send this command to control the position of the motor(Multi turn angle)
    RS485_MULTI_LOOP_ANGLE_CTL_2 = 0xA4, // #13. Host send this command to control the position of the motor(Multi turn angle).
    RS485_SINGLE_LOOP_ANGLE_CTL = 0xA5, // #14. Host send this command to control the position of the motor(Single turn angle)
    RS485_SINGLE_LOOP_ANGLE_CTL_2 = 0xA6, // #15. Host send this command to control the position of the motor(single turn angle).
    RS485_INC_ANGLE_CTL = 0xA7, // #16. Host send commands to control the increment angle of the motor.
    RS485_INC_ANGLE_CTL_2 = 0xA8, // #17. Host send commands to control the increment angle of the motor.
    RS485_READ_ENCODER_VALUE = 0x90, // #18. Host send commands to write acceleration to RAM.Parameter are invalid when power off.
    RS485_WRITE_CURRENT_POS = 0x19, // #19. Host send this command to set encoder offset.
    RS485_READ_MULTI_ANGLE_LOOP = 0x92, // #20. Write motor encoder current position to ROM as the initial position.
    RS485_CLEAR_MOTOR_ANGLE_LOOP = 0x93, // #21. This command clear motor multi turn and single turn data and set
    // current position as motor zero point. It’s invalid when power off.
    RS485_READ_SINGLE_ANGLE_LOOP = 0x94, // #22. Host send this command to read motor current single angle value.
    RS485_RESET_AND_WRITE_POS_TO_RAM = 0x95, // #23. Write the current position to RAM as the motor zero command.
    // Host send commands to set motor current position as the zero position, clear the loop info at the same time.
    RS485_READ_PID = 0x40, // #24. Host send this command to read current motor PID parameters.
    RS485_WRITE_PID_TO_RAM = 0x42, // #25. Host send this command to write PID parameters to RAM, parameter are invalid when power off.
    RS485_WRITE_PID_TO_ROM = 0x44, // #26. Host sent this command to write PID paramters to ROM, parameters are valid when power off.
    RS485_READ_MOTOR_INFO = 0x12 // #27. This command is to read driver type,motor type, hardware version and firmware version
};

}
}

#endif // SERVO_DEF_H
