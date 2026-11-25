#include "servo_can_controller.h"

namespace KtechServo {
namespace CanCtl {

ServoCanController::ServoCanController(std::string can_iface)
{
    m_can_ctl = std::make_unique<CanController>(can_iface);
}

bool ServoCanController::initialize(int bitrate)
{
    return m_can_ctl->openIface();
}

bool ServoCanController::isInit()
{
    return m_can_ctl->isOpen();
}

bool ServoCanController::startCan()
{
   return m_can_ctl->startCan();
}

bool ServoCanController::stopCan()
{
   m_can_ctl->closeIface();
   return m_can_ctl->stopCan();
}

bool ServoCanController::motorOn(int id)
{
    return sendAndCheckCommonResp(id, CAN_SINGLE_MOTOR_COMMAND::CAN_MOTOR_ON);
}

bool ServoCanController::motorOff(int id)
{
    return sendAndCheckCommonResp(id, CAN_SINGLE_MOTOR_COMMAND::CAN_MOTOR_OFF);
}

bool ServoCanController::motorStop(int id)
{
    return sendAndCheckCommonResp(id, CAN_SINGLE_MOTOR_COMMAND::CAN_MOTOR_STOP);
}

FullServoStatus ServoCanController::readAllState(int id)
{
    MotorStateAndErrorRespData state1;
    MotorStateRespData2 state2;
    MotorStateRespData3 state3;
    FullServoStatus out;

    bool status1 = this->sendCommonCommand(id, CAN_SINGLE_MOTOR_COMMAND::CAN_READ_MOTOR_STATE);
    if(status1){
        CanData data = this->readResp(id);
        status1 = data.status;
        if(status1){
            state1 = this->parseStateRespMsg(data);
        }
    }

    bool status2 = this->sendCommonCommand(id, CAN_SINGLE_MOTOR_COMMAND::CAN_READ_MOTOR_STATE_2);
    if(status2){
        CanData data = this->readResp(id);
        status2 = data.status;
        if(status2){
            state2 = this->parseStateRespMsg2(data);
        }
    }

    bool status3 = this->sendCommonCommand(id, CAN_SINGLE_MOTOR_COMMAND::CAN_READ_MOTOR_STATE_3);
    if(status3){
        CanData data = this->readResp(id);
        status3 = data.status;
        if(status3){
            state3 = this->parseStateRespMsg3(data);
        }
    }

    if(status1){
        out.temp = state1.temp;
        out.voltage = state1.voltage;
        out.error = state1.error_state;
    }
    if(status2){
        out.temp = state2.temp;
        out.torque_value = state2.torque;
        out.encoder_pos = state2.encoder_pos;
        out.speed = state2.speed;
    }
    if(status3){
        out.temp = state3.temp;
        out.phase_a = state3.phase_a;
        out.phase_b = state3.phase_b;
        out.phase_c = state3.phase_c;
    }


    return out;
}

FullServoStatus ServoCanController::readState1(int id)
{
    MotorStateAndErrorRespData state1;
    FullServoStatus out;

    bool status1 = this->sendCommonCommand(id, CAN_SINGLE_MOTOR_COMMAND::CAN_READ_MOTOR_STATE);
    if(status1){
        CanData data = this->readResp(id);
        status1 = data.status;
        if(status1){
            state1 = this->parseStateRespMsg(data);
        }
    }

    if(status1){
        out.temp = state1.temp;
        out.voltage = state1.voltage;
        out.error = state1.error_state;
    }

    return out;
}

FullServoStatus ServoCanController::readState2(int id)
{
    MotorStateRespData2 state2;
    FullServoStatus out;

    bool status2 = this->sendCommonCommand(id, CAN_SINGLE_MOTOR_COMMAND::CAN_READ_MOTOR_STATE_2);
    if(status2){
        CanData data = this->readResp(id);
        status2 = data.status;
        if(status2){
            state2 = this->parseStateRespMsg2(data);
        }
    }


    if(status2){
        out.temp = state2.temp;
        out.torque_value = state2.torque;
        out.encoder_pos = state2.encoder_pos;
        out.speed = state2.speed;
    }

    return out;
}

FullServoStatus ServoCanController::readState3(int id)
{
    MotorStateRespData3 state3;
    FullServoStatus out;

    bool status3 = this->sendCommonCommand(id, CAN_SINGLE_MOTOR_COMMAND::CAN_READ_MOTOR_STATE_3);
    if(status3){
        CanData data = this->readResp(id);
        status3 = data.status;
        if(status3){
            state3 = this->parseStateRespMsg3(data);
        }
    }

    if(status3){
        out.temp = state3.temp;
        out.phase_a = state3.phase_a;
        out.phase_b = state3.phase_b;
        out.phase_c = state3.phase_c;
    }


    return out;
}

bool ServoCanController::writePid(int id, PIDData pid, WRITE_TYPE type)
{
    PIDComData send_pid_data;
    if(type == WRITE_TYPE::WRITE_RAM){
        send_pid_data.com = CAN_WRITE_PID_TO_RAM;
    } else if(type == WRITE_TYPE::WRITE_ROM){
        send_pid_data.com = CAN_WRITE_PID_TO_ROM;
    }else {
        return false;
    }

    send_pid_data.pos_i_value = pid.pos_i_value;
    send_pid_data.pos_p_value = pid.pos_p_value;
    send_pid_data.speed_i_value = pid.speed_i_value;
    send_pid_data.speed_p_value = pid.speed_p_value;
    send_pid_data.torque_i_value = pid.torque_i_value;
    send_pid_data.torque_p_value = pid.torque_p_value;

    CanData can_data;
    can_data.data.resize(SERVO_CAN_MSG_LEN);
    std::memcpy(can_data.data.data(), (char*)&send_pid_data, SERVO_CAN_MSG_LEN);

    bool status = this->sendData(id, can_data);
    if(status){
        CanData resp_data = this->readResp(id);

        return resp_data.status;
    }

    return status;
}

PIDData ServoCanController::readPid(int id)
{
    PIDData out;
    CanData data;
    if(sendAndCheckDataResp(id, CAN_SINGLE_MOTOR_COMMAND::CAN_READ_PID, data)){
        PIDRespData pid_data = this->parsePidMsg(data);

        out.pos_i_value = pid_data.pos_i_value;
        out.pos_p_value = pid_data.pos_p_value;
        out.speed_i_value = pid_data.speed_i_value;
        out.speed_p_value = pid_data.speed_p_value;
        out.torque_i_value = pid_data.torque_i_value;
        out.torque_p_value = pid_data.torque_p_value;
        out.status = true;
    }

    return out;
}

bool ServoCanController::clearError(int id)
{
    return sendAndCheckCommonResp(id, CAN_SINGLE_MOTOR_COMMAND::CAN_CLEAR_MOTOR_ERROR_STATE);
}

ServoStatus ServoCanController::multiLoopAngleCtl(int id, float angle, int speed)
{
    ServoStatus out;
    CanData data;
    StatusResp_t resp_status;

    if(speed <= 0){
        MlAngleCtlComData1 com_data;
        com_data.com = CAN_SINGLE_MOTOR_COMMAND::CAN_MULTI_LOOP_ANGLE_CTL;

        com_data.angle_value = convertAngle(angle);

        CanData can_data;
        can_data.data.resize(SERVO_CAN_MSG_LEN);
        std::memcpy(can_data.data.data(), (char*)&com_data, SERVO_CAN_MSG_LEN);

        bool status = this->sendData(id, can_data);
        if(status){
            CanData resp_data = this->readResp(id);

            if(resp_data.status){
                resp_status = this->parseStatusMsg(resp_data);
                out.toqrue_value = resp_status.torque_value;
                out.status = true;
            }
        }
    }else {
        if(speed > 0xFFFF){
            speed = 0xFFFF;
        }

        MlAngleCtlComData2 com_data;
        com_data.com = CAN_SINGLE_MOTOR_COMMAND::CAN_MULTI_LOOP_ANGLE_CTL_2;

        com_data.angle_value = convertAngle(angle);
        com_data.speed = speed;
        CanData can_data;
        can_data.data.resize(SERVO_CAN_MSG_LEN);
        std::memcpy(can_data.data.data(), (char*)&com_data, SERVO_CAN_MSG_LEN);

        bool status = this->sendData(id, can_data);
        if(status){
            CanData resp_data = this->readResp(id);

            if(resp_data.status){
                resp_status = this->parseStatusMsg(resp_data);
                out.toqrue_value = resp_status.torque_value;
                out.status = true;
            }
        }
    }

    return out;
}

ServoStatus ServoCanController::singleLoopAngleCtl(int id, float angle, int speed)
{
    ServoStatus out;
    CanData data;
    StatusResp_t resp_status;

    if(speed <= 0){
        SlAngleCtlComData1 com_data;
        com_data.com = CAN_SINGLE_MOTOR_COMMAND::CAN_SINGLE_LOOP_ANGLE_CTL;
        com_data.spin_dir = (angle > 0);
        com_data.angle_value = convertAngle(angle);

        CanData can_data;
        can_data.data.resize(SERVO_CAN_MSG_LEN);
        std::memcpy(can_data.data.data(), (char*)&com_data, SERVO_CAN_MSG_LEN);

        bool status = this->sendData(id, can_data);
        if(status){
            CanData resp_data = this->readResp(id);

            if(resp_data.status){
                resp_status = this->parseStatusMsg(resp_data);
                out.toqrue_value = resp_status.torque_value;
                out.status = true;
            }
        }
    }else {
        SlAngleCtlComData2 com_data;
        com_data.com = CAN_SINGLE_MOTOR_COMMAND::CAN_SINGLE_LOOP_ANGLE_CTL_2;

        com_data.angle_value = convertAngle(angle);
        com_data.speed = speed;

        CanData can_data;
        can_data.data.resize(SERVO_CAN_MSG_LEN);
        std::memcpy(can_data.data.data(), (char*)&com_data, SERVO_CAN_MSG_LEN);

        bool status = this->sendData(id, can_data);
        if(status){
            CanData resp_data = this->readResp(id);

            if(resp_data.status){
                resp_status = this->parseStatusMsg(resp_data);
                out.toqrue_value = resp_status.torque_value;
                out.status = true;
            }
        }
    }

    return out;
}

ServoStatus ServoCanController::incAngle(int id, float angle, int speed)
{
    ServoStatus out;
    CanData data;
    StatusResp_t resp_status;

    if(speed <= 0){
        IncAngleCtlComData1 com_data;
        com_data.com = CAN_SINGLE_MOTOR_COMMAND::CAN_INC_ANGLE_CTL;

        com_data.angle_value = convertAngle(angle);

        CanData can_data;
        can_data.data.resize(SERVO_CAN_MSG_LEN);
        std::memcpy(can_data.data.data(), (char*)&com_data, SERVO_CAN_MSG_LEN);

        bool status = this->sendData(id, can_data);
        if(status){
            CanData resp_data = this->readResp(id);

            if(resp_data.status){
                resp_status = this->parseStatusMsg(resp_data);
                out.toqrue_value = resp_status.torque_value;
                out.status = true;
            }
        }
    }else {
        IncAngleCtlComData2 com_data;
        com_data.com = CAN_SINGLE_MOTOR_COMMAND::CAN_INC_ANGLE_CTL_2;

        com_data.angle_value = convertAngle(angle);
        com_data.speed = speed;

        CanData can_data;
        can_data.data.resize(SERVO_CAN_MSG_LEN);
        std::memcpy(can_data.data.data(), (char*)&com_data, SERVO_CAN_MSG_LEN);

        bool status = this->sendData(id, can_data);
        if(status){
            CanData resp_data = this->readResp(id);

            if(resp_data.status){
                resp_status = this->parseStatusMsg(resp_data);
                out.toqrue_value = resp_status.torque_value;
                out.status = true;
            }
        }
    }

    return out;
}

bool ServoCanController::readAccel(int id, int32_t &accel)
{
    bool status = false;
    accel = 0;
    status = this->sendCommonCommand(id, CAN_SINGLE_MOTOR_COMMAND::CAN_READ_ACCELERATION);
    if(status){
        CanData data = this->readResp(id);
        status = data.status;
        if(status){
            ReadAccelRespData resp_data = this->parseAccelMsg(data);
            accel = resp_data.accel_data;
        }
    }

    return status;
}

bool ServoCanController::writeAccel(int id, int32_t accel)
{
    WriteAccelComData com_data;
    com_data.com = CAN_SINGLE_MOTOR_COMMAND::CAN_WRITE_ACCELETATION;

    com_data.accel_data = accel;

    CanData can_data;
    can_data.data.resize(SERVO_CAN_MSG_LEN);
    std::memcpy(can_data.data.data(), (char*)&com_data, SERVO_CAN_MSG_LEN);

    bool status = this->sendData(id, can_data);
    if(status){
        CanData data = this->readResp(id);

        if(data.status){
            ReadAccelRespData resp_data = this->parseAccelMsg(data);
            return (int32_t)resp_data.accel_data == accel;
        }
    }

    return status;
}

ServoStatus ServoCanController::readMultiAngle(int id)
{
    ServoStatus out;
    bool status = false;
    status = this->sendCommonCommand(id, CAN_SINGLE_MOTOR_COMMAND::CAN_READ_MULTI_ANGLE_LOOP);
    if(status){
        CanData data = this->readResp(id);
        status = data.status;
        if(status){
            MALComData resp_data = this->parseMultiAngleRespMsg(data);
            // Добавлен костыль, так как по CAN не могут передаваться 8 байт данных
            int64_t angle = resp_data.angle;
            if (angle > 0xFFFFFFFFFFFFF){
                angle += 0xFF00000000000000;
            }
            out.angle = (float)angle * M_ANGLE_RATIO;
            out.status = true;
        }
    }
	
    return out;
}

ServoStatus ServoCanController::readSingleAngle(int id)
{
    ServoStatus out;
    bool status = false;
    status = this->sendCommonCommand(id, CAN_SINGLE_MOTOR_COMMAND::CAN_READ_SINGLE_ANGLE_LOOP);
    if(status){
        CanData data = this->readResp(id);
        status = data.status;
        if(status){
            SALComData resp_data = this->parseSingleAngleRespMsg(data);
            out.angle = convertAngle(resp_data.angle);
            out.status = true;
        }
    }

    return out;
}

bool ServoCanController::clearMotorAngle(int id)
{
    return sendAndCheckCommonResp(id, CAN_SINGLE_MOTOR_COMMAND::CAN_CLEAR_MOTOR_ANGLE_LOOP);
}

EncoderData ServoCanController::readEncoderData(int id)
{
    EncoderData out;
    bool status = false;
    status = this->sendCommonCommand(id, CAN_SINGLE_MOTOR_COMMAND::CAN_READ_ENCODER_VALUE);
    if(status){
        CanData data = this->readResp(id);
        status = data.status;
        if(status){
            EncRespData resp_data = this->parseEncDataMsg(data);
            out.encoder = resp_data.encoder_data;
            out.encoder_pos = resp_data.encoder_raw_pos;
            out.encoder_offset = resp_data.encoder_offset;
            out.status = true;
        }
    }

    return out;
}

bool ServoCanController::writeEncoderData(int id, EncoderData data)
{
    EncZeroPointComData send_enc_data;
    send_enc_data.com = CAN_WRITE_ENCODER_VALUE;
    send_enc_data.encoder_offset = data.encoder_offset;


    CanData can_data;
    can_data.data.resize(SERVO_CAN_MSG_LEN);
    std::memcpy(can_data.data.data(), (char*)&send_enc_data, SERVO_CAN_MSG_LEN);

    bool status = this->sendData(id, can_data);
    if(status){
        CanData resp_data = this->readResp(id);

        if(resp_data.status){
            EncZeroPointRespData resp_enc_data = parseEncOffsetDataMsg(resp_data);

            return resp_enc_data.encoder_offset == data.encoder_offset;
        }
    }

    return status;
}

bool ServoCanController::resetPos(int id)
{
    return sendAndCheckCommonResp(id, CAN_SINGLE_MOTOR_COMMAND::CAN_CLEAR_MOTOR_ANGLE_LOOP);
}

ServoStatus ServoCanController::setSpeed(int id, int speed)
{
    ServoStatus out;
    SpeedClCtlComData send_speed_data;
    send_speed_data.com = CAN_SINGLE_MOTOR_COMMAND::CAN_SPEED_CLOSED_LOOP_CTL;
    send_speed_data.speed_value= int((float)speed/SPEED_CTL_RATIO);


    CanData can_data;
    can_data.data.resize(SERVO_CAN_MSG_LEN);
    std::memcpy(can_data.data.data(), (char*)&send_speed_data, SERVO_CAN_MSG_LEN);

    bool status = this->sendData(id, can_data);
    if(status){
        CanData resp_data = this->readResp(id);

        if(resp_data.status){
            StatusResp_t resp_status_data = parseStatusMsg(resp_data);
            out.toqrue_value = resp_status_data.torque_value;
            out.status = true;
        }
    }

    return out;
}

ServoStatus ServoCanController::setTorque(int id, int torque_value)
{
    ServoStatus out;
    TorqueClCtlComData send_data;
    send_data.com = CAN_SINGLE_MOTOR_COMMAND::CAN_TORQUE_CLOSED_LOOP_CTL;
	send_data.torque_value = torque_value;


    CanData can_data;
    can_data.data.resize(SERVO_CAN_MSG_LEN);
    std::memcpy(can_data.data.data(), (char*)&send_data, SERVO_CAN_MSG_LEN);

    bool status = this->sendData(id, can_data);
    if(status){
        CanData resp_data = this->readResp(id);

        if(resp_data.status){
            StatusResp_t resp_status_data = parseStatusMsg(resp_data);
            out.toqrue_value = resp_status_data.torque_value;
            out.status = true;
        }
    }

    return out;
}

bool ServoCanController::sendCommonCommand(int id, int com)
{
    CanData can_data;
    can_data.id = STANDERT_ID_OFFSET + id;
    can_data.data = std::vector<uint8_t>(SERVO_CAN_MSG_LEN, 0);

    CommonCommandData common_com_data;
    common_com_data.com = com;

    memcpy(can_data.data.data(), (char*)&common_com_data, sizeof(CommonCommandData));

    return m_can_ctl->writeData(can_data);
}

bool ServoCanController::sendData(int id, CanData can_data)
{
    can_data.id = STANDERT_ID_OFFSET + id;

    return m_can_ctl->writeData(can_data);
}

CanData ServoCanController::readResp(int id, int timeout)
{
    int servo_id = STANDERT_ID_OFFSET + id;
    return m_can_ctl->readData(servo_id);
}

bool ServoCanController::sendAndCheckCommonResp(int id, int com)
{
    bool status = false;
    status = this->sendCommonCommand(id, com);
    if(status){
        CanData data = this->readResp(id);
        status = data.status;
        if(status){
            int recv_com = this->parseCommonResp(data);

            return  recv_com == com;
        }
    }

    return status;
}

bool ServoCanController::sendAndCheckDataResp(int id, int com, CanData &data)
{
    bool status = false;
    data.clear();
    status = this->sendCommonCommand(id, com);
    if(status){
        data = this->readResp(id);
        status = data.status;
    }
    return status;
}


// parse
int ServoCanController::parseCommonResp(CanData data)
{
    CommonRespData_t *resp = (CommonRespData_t*)&data.data[0];
    int com = resp->com;

    return com;
}

StatusResp_t ServoCanController::parseStatusMsg(CanData data)
{
    StatusResp_t out;
    out = *(StatusResp_t*)&data.data[0];

    return out;
}

PIDRespData ServoCanController::parsePidMsg(CanData data)
{
    PIDRespData out;
    out = *(PIDRespData*)&data.data[0];

    return out;
}

ReadAccelRespData ServoCanController::parseAccelMsg(CanData data)
{
    ReadAccelRespData out;
    out = *(ReadAccelRespData*)&data.data[0];

    return out;
}

EncRespData ServoCanController::parseEncDataMsg(CanData data)
{
    EncRespData out;
    out = *(EncRespData*)&data.data[0];

    return out;
}

EncZeroPointRespData ServoCanController::parseEncOffsetDataMsg(CanData data)
{
    EncZeroPointRespData out;
    out = *(EncZeroPointRespData*)&data.data[0];

    return out;
}

MotorStateAndErrorRespData ServoCanController::parseStateRespMsg(CanData data)
{
    MotorStateAndErrorRespData out;
    out = *(MotorStateAndErrorRespData*)&data.data[0];

    return out;
}

MotorStateRespData2 ServoCanController::parseStateRespMsg2(CanData data)
{
    MotorStateRespData2 out;
    out = *(MotorStateRespData2*)&data.data[0];

    return out;
}


MotorStateRespData3 ServoCanController::parseStateRespMsg3(CanData data)
{
    MotorStateRespData3 out;
    out = *(MotorStateRespData3*)&data.data[0];

    return out;
}

MALComData ServoCanController::parseMultiAngleRespMsg(CanData data)
{
    MALComData out;
    out = *(MALComData*)&data.data[0];

    return out;
}

SALComData ServoCanController::parseSingleAngleRespMsg(CanData data)
{
    SALComData out;
    out = *(SALComData*)&data.data[0];

    return out;
}



} // namespace CanCtl
} // namespace KtechServo
