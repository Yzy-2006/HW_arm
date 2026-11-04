#include <cassert>
#include <cmath>
#include "servo_control/transmit.h"

gripperCmd left_gripper_cmd, right_gripper_cmd;
gripperFdb left_gripper_fdb, right_gripper_fdb;
extern int ecat_phys, expectedWKC;
extern volatile int wkc;
extern MotorBlock pdo_tx[CAN_CHANNELS][MOTORS_PER_CH];
void EtherCAT_Data_Get();

int main() {
    // Never initialize SOEM or open a network interface. Inspect in-memory PDOs.
    ecat_phys = -1;
    ServoStruct.CAN_board_num = 2;
    ServoStruct.CAN_boards.resize(2);
    for (int ch = 0; ch < 2; ++ch) {
        auto& board = ServoStruct.CAN_boards[ch];
        board.channel_idx = ch;
        board.JntNum_onCan = 8;
        board.can_jnt_ret_msg.resize(8);
        for (int j = 0; j < 8; ++j) {
            board.motor_slot.push_back(j);
            board.can_jnt_ID.push_back(j + 1);
            board.can_jnt_direComp.push_back(1);
            board.can_jnt_des_msg.push_back({2, 0, 0, 0, 100, 12});
        }
    }
    arm_safety.limits.lower.fill(-3);
    arm_safety.limits.upper.fill(3);
    for (auto& f : arm_safety.feedback) f = {0, 0, arm_safety_now(), 0};
    EtherCAT_Command_Set();
    assert(arm_safety.fault.empty());
    assert(pdo_tx[0][0].data[0] != 0);  // Healthy position gain was encoded.

    // A fault on the last joint must inhibit BOTH arms in the same output cycle.
    ServoStruct.CAN_boards[1].can_jnt_des_msg[6].joint_tor = -44114;
    EtherCAT_Command_Set();
    assert(!arm_safety.fault.empty());
    for (int ch = 0; ch < 2; ++ch) {
        for (int j = 0; j < 8; ++j) {
            const auto& block = pdo_tx[ch][j];
            const int kp = (block.data[0] << 7) | (block.data[1] >> 1);
            const int tor = ((block.data[6] & 15) << 8) | block.data[7];
            assert(kp == 0);
            assert(tor == float_to_uint(0, T_MIN, T_MAX, 12));
        }
    }

    MotorBlock input[CAN_CHANNELS][MOTORS_PER_CH]{};
    ecat_phys = 1;
    ec_slave[1].inputs = reinterpret_cast<uint8*>(input);
    expectedWKC = wkc = 3;
    auto& block = input[0][0];
    block.id = 1; block.dlc = 7; block.data[0] = 0x20;
    block.data[1] = 128; block.data[3] = 128;
    arm_safety.feedback[0].received = 123;
    EtherCAT_Data_Get();
    assert(arm_safety.feedback[0].received != 123);
    arm_safety.feedback[0].received = 123;
    block.dlc = 3;
    EtherCAT_Data_Get();
    assert(arm_safety.feedback[0].received == 123);
    block.dlc = 7; block.id = 2;
    EtherCAT_Data_Get();
    assert(arm_safety.feedback[0].received == 123);
    block.id = 1; block.data[0] = 0x40;  // ACK2 has no complete q/v sample.
    EtherCAT_Data_Get();
    assert(arm_safety.feedback[0].received == 123);
    block.data[0] = 0x20; wkc = 0;
    EtherCAT_Data_Get();
    assert(arm_safety.feedback[0].received == 123);
}
