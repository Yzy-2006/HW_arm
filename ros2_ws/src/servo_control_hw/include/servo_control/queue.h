//
// Created by bismarck on 11/18/22.
//

#ifndef SOEM_QUEUE_H
#define SOEM_QUEUE_H

#include <boost/lockfree/spsc_queue.hpp>
#include <memory>
#include <atomic>
#include <thread>

#include "servo_control/transmit.h"

using boost::lockfree::spsc_queue;
using boost::lockfree::capacity;

typedef std::shared_ptr<MotorBlock> MotorBlock_ptr;

extern spsc_queue<MotorBlock_ptr, capacity<30>> messages[SLAVE_NUMBER];
extern std::atomic<bool> running;
extern std::thread runThread;

#endif //SOEM_QUEUE_H
