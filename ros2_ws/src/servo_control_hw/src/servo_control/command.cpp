//
// Created by bismarck on 11/19/22.
//

#include "servo_control/command.h"

void sendToQueue(int slaveId, const MotorBlock_ptr& msg) {
    if (messages[slaveId].write_available()) {
        messages[slaveId].push(msg);
    } else {
        std::cout << "Queue Fulled, Waiting For Command Executing\n";
        while(messages[slaveId].push(msg)) sleep(1);
    }
}

unsigned help(const std::vector<std::string> &) {
    std::cout << "Available Commands:\n"
        << "\tMotorIdGet <SlaveId>\n"
        << "\tMotorIdSet <SlaveId> <MotorId> <NewMotorId>\n"
        << "\tMotorSpeedSet <SlaveId> <PassAge> <MotorId> <Speed>(0) <Current>(500) <AckStatus>(2)\n"
        << "\tMotorPositionSet <SlaveId> <PassAge> <MotorId> <Position>(0) <Speed>(50) <Current>(500) <AckStatus>(2)\n";
    return 0;
}

unsigned motorIdGet(const std::vector<std::string> & input) {
    int slaveId;
    switch (input.size()-1) {
        case 1:
            slaveId = std::stoi(input[1]);
            break;
        default:
            std::cout << "Command format error\n" <<
                      "\tShould be \"MotorIdGet <SlaveId>\"\n";
            return 1;
    }
    MotorBlock_ptr msg(new MotorBlock);
    MotorIDReading(msg.get());
    sendToQueue(slaveId, msg);
    return 0;
}

unsigned motorIdSet(const std::vector<std::string> & input) {
    int slaveId;
    int motor_id,  motor_id_new;
    switch (input.size()-1) {
        case 3:
            slaveId = std::stoi(input[1]);
            motor_id = std::stoi(input[2]);
            motor_id_new = std::stoi(input[3]);
            break;
        default:
            std::cout << "Command format error\n" <<
                      "\tShould be \"MotorIdSet <SlaveId> <OldMotorId> <NewMotorId>\"\n";
            return 1;
    }
    MotorBlock_ptr msg(new MotorBlock);
    MotorIDSetting(msg.get(), motor_id, motor_id_new);
    sendToQueue(slaveId, msg);
    return 0;
}

unsigned motorSpeedSet(const std::vector<std::string> & input) {
    int slaveId;
    int motor_id ;
    float spd = 0;
    uint8_t passage;
    uint16_t cur = 500;
    uint8_t ack_status = 2;
    switch (input.size()-1) {
        case 6:
            ack_status = std::stoi(input[6]);
        case 5:
            cur = std::stoi(input[5]);
        case 4:
            spd = std::stof(input[4]);
        case 3:
            slaveId = std::stoi(input[1]);
            passage = std::stoi(input[2]);
            motor_id = std::stoi(input[3]);
            break;
        default:
            std::cout << "Command format error\n" <<
                      "\tShould be \"MotorSpeedSet <SlaveId> <PassAge> <OldMotorId> <Speed>(0) <Current>(500) <AckStatus>(2)\"\n";
            return 1;
    }
    MotorBlock_ptr msg(new MotorBlock);
    set_motor_speed(msg.get(), motor_id, spd, cur, ack_status);
    sendToQueue(slaveId, msg);
    return 0;
}

unsigned motoPositionSet(const std::vector<std::string> & input) {
    int slaveId;
    int motor_id;
    float pos = 0;
    uint8_t passage;
    uint16_t spd = 50;
    uint16_t cur = 500;
    uint8_t ack_status = 2;
    switch (input.size()-1) {
        case 7:
            ack_status = std::stoi(input[7]);
        case 6:
            cur = std::stoi(input[6]);
        case 5:
            spd = std::stoi(input[5]);
        case 4:
            pos = std::stof(input[4]);
        case 3:
            slaveId = std::stoi(input[1]);
            passage = std::stoi(input[2]);
            motor_id = std::stoi(input[3]);
            break;
        default:
            std::cout << "Command format error\n" <<
                      "\tShould be \"MotorPositionSet <SlaveId> <PassAge> <OldMotorId> <Position>(0) <Speed>(50) <Current>(500) <AckStatus>(2)\"\n";
            return 1;
    }
    MotorBlock_ptr msg(new MotorBlock);
    set_motor_position(msg.get(), motor_id, pos, spd, cur, ack_status);
    sendToQueue(slaveId, msg);
    return 0;
}
