#include "servo_rs485_controller.h"

namespace KtechServo {
namespace Rs485Ctl {

ServoRs485Controller::ServoRs485Controller(std::string rs485_iface)
{
    m_rs485_iface = rs485_iface;
    m_rs485_ctl = std::make_unique<Rs485Controller>();
}

bool ServoRs485Controller::initialize(int bitrate)
{
    return m_rs485_ctl->initializePort(m_rs485_iface, bitrate);
}

bool ServoRs485Controller::isInit()
{
    return m_rs485_ctl->isOpen();
}

bool ServoRs485Controller::deinitialize()
{
    if(m_rs485_ctl->isOpen()){
        return m_rs485_ctl->closePort();
    }

    return true;
}

bool ServoRs485Controller::motorOn(int id)
{
    return this->sendAndCheckCommonResp(id, RS485_SINGLE_MOTOR_COMMAND::RS485_MOTOR_ON);
}

bool ServoRs485Controller::motorOff(int id)
{
    return this->sendAndCheckCommonResp(id, RS485_SINGLE_MOTOR_COMMAND::RS485_MOTOR_OFF);
}

bool ServoRs485Controller::motorStop(int id)
{
    return this->sendAndCheckCommonResp(id, RS485_SINGLE_MOTOR_COMMAND::RS485_MOTOR_STOP);
}

FullServoStatus ServoRs485Controller::readAllState(int id)
{
    FullServoStatus out;
    FullServoStatus state1 = this->readState1(id);
    FullServoStatus state2 = this->readState2(id);
    FullServoStatus state3 = this->readState3(id);

    if(state1.status){
        out.temp = state1.temp;
        out.voltage = state1.voltage;
        out.error = state1.error;
        out.motor_state = state1.motor_state;
    }
    if(state2.status){
        out.temp = state2.temp;
        out.torque_value = state2.torque_value;
        out.encoder_pos = state2.encoder_pos;
        out.speed = state2.speed;
    }
    if(state3.status){
        out.temp = state3.temp;
        out.phase_a = state3.phase_a;
        out.phase_b = state3.phase_b;
        out.phase_c = state3.phase_c;
    }

    return out;
}

FullServoStatus ServoRs485Controller::readState1(int id)
{
    FullServoStatus out;
    int com = RS485_SINGLE_MOTOR_COMMAND::RS485_CLEAR_MOTOR_ERROR_STATE;

    bool status = this->sendCommonCommand(id, com);

    if(status){
        std::vector<uint8_t> resp = this->readResp();
        MotorStateAndErrorRespData data_frame;
        if(!this->parseFrame<MotorStateAndErrorRespData>(data_frame, com, resp)){
            return out;
        }

        out.temp = (int)data_frame.temp;
        out.error = data_frame.error_state;
        out.voltage = data_frame.voltage;
        out.motor_state = data_frame.motor_state;
        out.status = true;
    }

    return out;
}

FullServoStatus ServoRs485Controller::readState2(int id)
{
    FullServoStatus out;
    int com = RS485_SINGLE_MOTOR_COMMAND::RS485_READ_MOTOR_STATE_2;

    bool status = this->sendCommonCommand(id, com);

    if(status){
        std::vector<uint8_t> resp = this->readResp();
        MotorStateRespData2 data_frame;
        if(!this->parseFrame<MotorStateRespData2>(data_frame, com, resp)){
            return out;
        }

        out.temp = (int)data_frame.temp;
        out.torque_value = data_frame.torque;
        out.speed = data_frame.speed;
        out.encoder_pos = data_frame.encoder_pos;
        out.status = true;
    }

    return out;
}

FullServoStatus ServoRs485Controller::readState3(int id)
{
    FullServoStatus out;
    int com = RS485_SINGLE_MOTOR_COMMAND::RS485_READ_MOTOR_STATE_3;

    bool status = this->sendCommonCommand(id, com);

    if(status){
        std::vector<uint8_t> resp = this->readResp();
        MotorStateRespData3 data_frame;
        if(!this->parseFrame<MotorStateRespData3>(data_frame, com, resp)){
            return out;
        }

        out.temp = (int)data_frame.temp;
        out.phase_a = data_frame.phase_a;
        out.phase_b = data_frame.phase_b;
        out.phase_c = data_frame.phase_c;
        out.status = true;
    }

    return out;
}

bool ServoRs485Controller::writePid(int id, PIDData pid, WRITE_TYPE type)
{
    // not supported in the class
    return true;
}

PIDData ServoRs485Controller::readPid(int id)
{
    // not supported in the class
    PIDData out;

    return out;
}

bool ServoRs485Controller::clearError(int id)
{
    FullServoStatus out;
    int com = RS485_SINGLE_MOTOR_COMMAND::RS485_CLEAR_MOTOR_ERROR_STATE;

    bool status = this->sendCommonCommand(id, com);

    if(status){
        std::vector<uint8_t> resp = this->readResp();
        MotorStateAndErrorRespData data_frame;
        if(!this->parseFrame<MotorStateAndErrorRespData>(data_frame, com, resp)){
            return false;
        }

        out.temp = (int)data_frame.temp;
        out.error = data_frame.error_state;
        out.voltage = data_frame.voltage;
        out.motor_state = data_frame.motor_state;
        out.status = true;
    }

    return out.status;
}

ServoStatus ServoRs485Controller::multiLoopAngleCtl(int id, float angle, int speed)
{
    ServoStatus out;

    if(speed <= 0){
        int com = RS485_SINGLE_MOTOR_COMMAND::RS485_MULTI_LOOP_ANGLE_CTL;

        MlAngleCtlComData1 data_frame;
        MotorStateRespData2 data_resp;

        data_frame.angle_value = angle;
        data_frame.data_checksum = this->calcChecksum((uint8_t*)&data_frame, sizeof (data_frame) - 1);

        out = this->sendComAndGetServoState<MlAngleCtlComData1, MotorStateRespData2>(id, com, data_frame, data_resp);

    }else {
        int com = RS485_SINGLE_MOTOR_COMMAND::RS485_MULTI_LOOP_ANGLE_CTL_2;

        MlAngleCtlComData2 data_frame;
        MotorStateRespData2 data_resp;

        data_frame.angle_value = convertAngle(angle);
        data_frame.speed = int((float)speed/SPEED_CTL_RATIO);
        data_frame.data_checksum = this->calcChecksum((uint8_t*)&data_frame, sizeof (data_frame) - 1);

        out = this->sendComAndGetServoState<MlAngleCtlComData2, MotorStateRespData2>(id, com, data_frame, data_resp);

    }

    return out;
}

ServoStatus ServoRs485Controller::singleLoopAngleCtl(int id, float angle, int speed)
{
    ServoStatus out;

    if(speed <= 0){
        int com = RS485_SINGLE_MOTOR_COMMAND::RS485_SINGLE_LOOP_ANGLE_CTL;

        SlAngleCtlComData1 data_frame;
        MotorStateRespData2 data_resp;

        data_frame.angle_value = angle;
        data_frame.data_checksum = this->calcChecksum((uint8_t*)&data_frame, sizeof (data_frame) - 1);

        out = this->sendComAndGetServoState<SlAngleCtlComData1, MotorStateRespData2>(id, com, data_frame, data_resp);

    }else {
        int com = RS485_SINGLE_MOTOR_COMMAND::RS485_SINGLE_LOOP_ANGLE_CTL_2;

        SlAngleCtlComData2 data_frame;
        MotorStateRespData2 data_resp;

        data_frame.angle_value = convertAngle(angle);
        data_frame.speed = int((float)speed/SPEED_CTL_RATIO);
        data_frame.data_checksum = this->calcChecksum((uint8_t*)&data_frame, sizeof (data_frame) - 1);

        out = this->sendComAndGetServoState<SlAngleCtlComData2, MotorStateRespData2>(id, com, data_frame, data_resp);
    }

    return out;
}

ServoStatus ServoRs485Controller::incAngle(int id, float angle, int speed)
{
    ServoStatus out;

    if(speed <= 0){
        int com = RS485_SINGLE_MOTOR_COMMAND::RS485_INC_ANGLE_CTL;

        IncAngleCtlComData1 data_frame;
        MotorStateRespData2 data_resp;

        data_frame.angle_value = convertAngle(angle);
        data_frame.data_checksum = this->calcChecksum((uint8_t*)&data_frame, sizeof (data_frame) - 1);

        out = this->sendComAndGetServoState<IncAngleCtlComData1, MotorStateRespData2>(id, com, data_frame, data_resp);

    }else {
        int com = RS485_SINGLE_MOTOR_COMMAND::RS485_INC_ANGLE_CTL_2;

        IncAngleCtlComData2 data_frame;
        MotorStateRespData2 data_resp;

        data_frame.angle_value = convertAngle(angle);
        data_frame.speed = int((float)speed/SPEED_CTL_RATIO);
        data_frame.data_checksum = this->calcChecksum((uint8_t*)&data_frame, sizeof (data_frame) - 1);

        out = this->sendComAndGetServoState<IncAngleCtlComData2, MotorStateRespData2>(id, com, data_frame, data_resp);
    }

    return out;
}

bool ServoRs485Controller::readAccel(int id, int32_t &accel)
{
    // not supported
    return false;
}

bool ServoRs485Controller::writeAccel(int id, int32_t accel)
{
    // not supported
    return false;
}

ServoStatus ServoRs485Controller::readMultiAngle(int id)
{
    ServoStatus out;
    int com = RS485_SINGLE_MOTOR_COMMAND::RS485_READ_MULTI_ANGLE_LOOP;

    bool status = this->sendCommonCommand(id, com);

    if(status){
        std::vector<uint8_t> resp = this->readResp();
        MALRespData data_frame;
        if(!this->parseFrame<MALRespData>(data_frame, com, resp)){
            return out;
        }

        out.angle = data_frame.angle;
        out.status = true;
    }

    return out;
}

ServoStatus ServoRs485Controller::readSingleAngle(int id)
{
    ServoStatus out;
    int com = RS485_SINGLE_MOTOR_COMMAND::RS485_READ_SINGLE_ANGLE_LOOP;

    bool status = this->sendCommonCommand(id, com);

    if(status){
        std::vector<uint8_t> resp = this->readResp();
        SALRespData data_frame;
        if(!this->parseFrame<SALRespData>(data_frame, com, resp)){
            return out;
        }

        out.angle = data_frame.angle;
        out.status = true;
    }

    return out;
}

bool ServoRs485Controller::clearMotorAngle(int id)
{
    return this->sendAndCheckCommonResp(id, RS485_SINGLE_MOTOR_COMMAND::RS485_CLEAR_MOTOR_ANGLE_LOOP);
}

EncoderData ServoRs485Controller::readEncoderData(int id)
{
    EncoderData out;

    int com = RS485_SINGLE_MOTOR_COMMAND::RS485_READ_ENCODER_VALUE;
    float status = this->sendCommonCommand(id, com);

    if(status){
        std::vector<uint8_t> resp = this->readResp();
        EncRespData data_frame;
        if(!this->parseFrame<EncRespData>(data_frame, com, resp)){
            return out;
        }


        out.encoder = data_frame.encoder_data;
        out.encoder_pos = data_frame.encoder_raw_pos;
        out.encoder_offset = data_frame.encoder_offset;
        out.status = true;
    }

    return out;
}

bool ServoRs485Controller::writeEncoderData(int id, EncoderData data)
{
    // Не поддерживается
    return false;
}

bool ServoRs485Controller::resetPos(int id)
{
    return sendAndCheckCommonResp(id, RS485_SINGLE_MOTOR_COMMAND::RS485_CLEAR_MOTOR_ANGLE_LOOP);
}

ServoStatus ServoRs485Controller::setSpeed(int id, int speed)
{
    ServoStatus out;

    if(speed == 0){
        return out;
    }

    int com = RS485_SINGLE_MOTOR_COMMAND::RS485_SPEED_CLOSED_LOOP_CTL;

    SpeedClCtlComData data_frame;
    MotorStateRespData2 data_resp;

    data_frame.speed_value = int((float)speed/SPEED_CTL_RATIO);
    data_frame.data_checksum = this->calcChecksum((uint8_t*)&data_frame, sizeof (data_frame) - 1);

    out = this->sendComAndGetServoState<SpeedClCtlComData, MotorStateRespData2>(id, com, data_frame, data_resp);

    return out;
}

ServoStatus ServoRs485Controller::setTorque(int id, int torque_value)
{
    ServoStatus out;

    int com = RS485_SINGLE_MOTOR_COMMAND::RS485_TORQUE_CLOSED_LOOP_CTL;

    TorqueClCtlComData data_frame;
    MotorStateRespData2 data_resp;

    data_frame.torque_value = torque_value;
    data_frame.data_checksum = this->calcChecksum((uint8_t*)&data_frame, sizeof (data_frame) - 1);

    out = this->sendComAndGetServoState<TorqueClCtlComData, MotorStateRespData2>(id, com, data_frame, data_resp);

    return out;
}

bool ServoRs485Controller::sendCommonCommand(int id, int com)
{
    std::vector<uint8_t> data;
    this->createCommandFrame(data, id, com, 0);

    return m_rs485_ctl->writeData(data);
}

bool ServoRs485Controller::sendData(int id, int com, std::vector<uint8_t> &data)
{
    std::vector<uint8_t> out_packet;
    this->createCommandFrame(out_packet, id, com, data.size() - 1);

    out_packet.insert(out_packet.end(), data.begin(), data.end());

	return m_rs485_ctl->writeData(out_packet);
}

void ServoRs485Controller::createCommandFrame(std::vector<uint8_t> &data, int id, int com, int len)
{
    data.clear();
    CommandFrame  com_frame;

    com_frame.id = id;
    com_frame.command = com;
    com_frame.data_lenght = len;
    com_frame.checksum = this->calcChecksum((uint8_t*)&com_frame, sizeof(CommandFrame) - 1);

    data.resize(RS485_COMMAND_FRAME_LEN, 0);
    memcpy(data.data(), (char*)&com_frame, sizeof(CommandFrame));
}

uint8_t ServoRs485Controller::calcChecksum(std::vector<uint8_t> &data, int size)
{
    uint32_t checksum = 0;
    for(int i = 0; i < size; i++)
    {
        checksum = data[i];
    }

    return (uint8_t)checksum;
}

uint8_t ServoRs485Controller::calcChecksum(uint8_t *data, int size)
{
    uint32_t checksum = 0;
    for(int i = 0; i < size; i++)
    {
        checksum += data[i];
    }

    return (uint8_t)checksum;
}

std::vector<uint8_t> ServoRs485Controller::readResp(int timeout)
{
    std::vector<uint8_t> out;
    bool status = m_rs485_ctl->readData(out, timeout);

    return out;
}

bool ServoRs485Controller::sendAndCheckCommonResp(int id, int com)
{
    bool status = false;
    status = this->sendCommonCommand(id, com);
    if(status){
        std::vector<uint8_t> resp_data = this->readResp(id);

        if(resp_data.size() > 0 && resp_data.size() == sizeof (CommandFrame)){
            CommandFrame resp = this->parseDataResp<CommandFrame>(resp_data);
            uint8_t calc_checksum = this->calcChecksum((uint8_t*)&resp, sizeof (CommandFrame) -1);
            return  (resp.command == com) && calc_checksum == resp.checksum;
        } else {
            status = false;
        }
    }

    return status;
}


template<typename RespStruct>
bool ServoRs485Controller::parseFrame(RespStruct &out_struct, int com, std::vector<uint8_t> &frame)
{
    if(frame.size() < sizeof (CommandFrame)){
        return false;
    }

    CommandFrame com_frame = this->parseDataResp<CommandFrame>(frame, 0);
    uint8_t calc_com_checksum = this->calcChecksum((uint8_t*)&com_frame, sizeof(CommandFrame) - 1);
    if(com_frame.checksum != calc_com_checksum){
        return false;
    }

    if(com_frame.command != com){
        return false;
    }

    if(frame.size() < sizeof (CommandFrame) + sizeof(RespStruct)){
        return false;
    }

    RespStruct data_frame = this->parseDataResp<RespStruct>(frame, sizeof(CommandFrame));
    uint8_t calc_data_checksum = this->calcChecksum((uint8_t*)&data_frame, sizeof(RespStruct) - 1);
    if(data_frame.data_checksum != calc_data_checksum){
        return false;
    }

    out_struct = data_frame;

    return true;
}

template<typename DataStruct, typename RespStruct>
ServoStatus ServoRs485Controller::sendComAndGetServoState(int id, int com, const DataStruct &data_frame, RespStruct &data_resp)
{
    ServoStatus out;

    this->sendAndGetDataResp<DataStruct, RespStruct>(id, com, data_frame, data_resp);

    out.temp = (int)data_resp.temp;
    out.toqrue_value = data_resp.torque;
    out.speed = data_resp.speed;
    out.encoder_pos = data_resp.encoder_pos;
    out.status = true;

    return out;
}

template<typename DataStruct, typename RespStruct>
bool ServoRs485Controller::sendAndGetDataResp(int id, int com, const DataStruct &data_frame, RespStruct &data_resp)
{
    std::vector<uint8_t> data_array;
	data_array.resize(sizeof(data_frame));
    std::memcpy(data_array.data(), (char*)&data_frame, sizeof (data_frame));

    bool ret = this->sendData(id, com, data_array);

    if(!ret){
        return false;
    }

    std::vector<uint8_t> resp = this->readResp();
    if(!this->parseFrame<MotorStateRespData2>(data_resp, com, resp)){
        return false;
    }

    return true;
}

template<typename RespStruct>
RespStruct ServoRs485Controller::parseDataResp(std::vector<uint8_t> &data, int point)
{
    RespStruct out;
    out = *(RespStruct*)&data[point];

    return out;
}





}
}
