//
// Created by bismarck on 11/19/22.
//

#ifndef MASTERSTACK_COMMAND_H
#define MASTERSTACK_COMMAND_H

#include <iostream>
#include <vector>
#include <unistd.h>
#include <chrono>
#include "servo_control/queue.h"

extern "C" {
#include "servo_control/config.h"
#include "servo_control/motor_control.h"
#include "servo_control/transmit.h"
}

unsigned help(const std::vector<std::string> &);
unsigned motorIdGet(const std::vector<std::string> & input);
unsigned motorIdSet(const std::vector<std::string> & input);
unsigned motorSpeedSet(const std::vector<std::string> & input);
unsigned motoPositionSet(const std::vector<std::string> & input);
void sendToQueue(int slaveId, const MotorBlock_ptr& msg);

#endif //MASTERSTACK_COMMAND_H
