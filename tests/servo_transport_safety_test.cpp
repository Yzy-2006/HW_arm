#include <cassert>
#include <cmath>
#include "servo_control/transmit.h"
#include "servo_control/motorFeedbackQuery.h"
#include "servo_control/motorZeroSession.h"

// Query transport tests inspect PDO memory without opening any interface.
extern int ecat_phys;
static MotorBlock last_sent[CAN_CHANNELS][MOTORS_PER_CH]{};
extern "C" int __wrap_ec_send_processdata(void) {
    if (ecat_phys > 0 && ec_slave[ecat_phys].outputs)
        std::memcpy(last_sent, ec_slave[ecat_phys].outputs, ETC_PDO_SIZE);
    return 1;
}
extern "C" int __wrap_ec_receive_processdata(int) { return 3; }

gripperCmd left_gripper_cmd, right_gripper_cmd;
gripperFdb left_gripper_fdb, right_gripper_fdb;
extern int ecat_phys, expectedWKC;
extern volatile int wkc;
extern MotorBlock pdo_tx[CAN_CHANNELS][MOTORS_PER_CH];
void EtherCAT_Data_Get();

MotorBlock raw_angle(int id, float degrees) {
    MotorBlock reply{};
    reply.id = id; reply.dlc = 6; reply.data[0] = 0xa0; reply.data[1] = 1;
    uint32_t bits;
    std::memcpy(&bits, &degrees, sizeof(bits));
    for (int i = 0; i < 4; ++i) reply.data[2 + i] = bits >> (24 - i * 8);
    return reply;
}

void test_zero_session() {
    using Phase = MotorZeroSession::Phase;
    const auto left = MotorZeroTarget::from_name("left_shoulder_roll_joint");
    const auto right = MotorZeroTarget::from_name("right_wrist_pitch_joint");
    assert(left.channel == 0 && left.motor_id == 2);
    assert(right.channel == 1 && right.motor_id == 6);
    for (const char* name : {"", "all", "left", "left_gripper", "left_joint_8"}) {
        bool rejected = false;
        try { MotorZeroTarget::from_name(name); } catch (const std::invalid_argument&) { rejected = true; }
        assert(rejected);
    }
    auto prepare = [](MotorZeroSession& session) {
        for (int i = 0; i < 12 && session.phase == Phase::measuring; ++i) {
            const auto query = session.next(i * .02);
            assert(query.id == 2 && query.dlc == 2 && query.data[0] == 0xe0);
            session.receive(raw_angle(2, -180), true, i * .02);
        }
        assert(session.phase == Phase::send_zero && !session.sent);
    };
    MotorZeroSession session(2, 0);
    prepare(session);
    const auto zero = session.next(.3);
    assert(zero.id == 0x7ff && zero.dlc == 4);
    assert(zero.data[0] == 0 && zero.data[1] == 2 && zero.data[2] == 0 && zero.data[3] == 3);
    assert(session.next(.31).dlc == 0);
    MotorBlock ack{};
    ack.id = 0x7ff; ack.dlc = 4; ack.data[1] = 3; ack.data[2] = 1; ack.data[3] = 3;
    session.receive(ack, true, .32);
    assert(!session.acknowledged);
    ack.data[1] = 2;
    session.receive(ack, false, .33);
    assert(!session.acknowledged);
    ack.dlc = 3;
    session.receive(ack, true, .34);
    assert(!session.acknowledged);
    ack.dlc = 4;
    session.receive(ack, true, .4);
    assert(session.acknowledged && session.phase == Phase::verifying);
    for (int n = 0; n < 7 && session.phase != Phase::done; ++n) {
        assert(session.next(.5 + n * .02).data[0] != 0);  // Query, never another zero.
        session.receive(raw_angle(2, .01), true, .5 + n * .02);
    }
    assert(session.phase == Phase::done && session.before_deg == -180);
    assert(session.next(1).dlc == 0);

    MotorZeroSession moving(2, 0);
    moving.receive(raw_angle(2, 0), true, 0);
    moving.receive(raw_angle(2, 1), true, .02);
    assert(moving.phase == Phase::failed && !moving.sent);
    MotorZeroSession missing(2, 0);
    missing.receive(raw_angle(3, 0), true, 0);
    missing.next(2.1);
    assert(missing.phase == Phase::failed && !missing.sent);
    MotorZeroSession no_ack(2, 0);
    prepare(no_ack);
    no_ack.next(.3);
    no_ack.next(2.4);
    assert(no_ack.phase == Phase::failed && no_ack.sent && !no_ack.acknowledged);
    MotorZeroSession not_zero(2, 0);
    prepare(not_zero);
    not_zero.next(.3);
    not_zero.receive(ack, true, .4);
    not_zero.receive(raw_angle(2, -180), true, .5);
    not_zero.next(2.5);
    assert(not_zero.phase == Phase::failed);
}

int main() {
    test_zero_session();
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

    {
        const auto saved_guard = arm_safety;
        arm_safety.limits.feedback_speed_trip_samples = 3;
        for (int sample = 1; sample <= 3; ++sample) {
            arm_safety.feedback[5] = {0, 2, arm_safety_now(), 0};
            EtherCAT_Command_Set();
            for (int ch = 0; ch < 2; ++ch) {
                for (int j = 0; j < 7; ++j) {
                    const auto& data = pdo_tx[ch][j].data;
                    const int kp = (data[0] << 7) | (data[1] >> 1);
                    assert((kp == 0) == (sample == 3));
                }
            }
            if (sample < 3) {
                assert(arm_safety.fault.empty() && arm_safety.ready(arm_safety_now()));
                for (int check = 0; check < 5; ++check) EtherCAT_Command_Set();
                assert(arm_safety.fault.empty());
            }
        }
        assert(arm_safety.fault.find("feedback_speed_joint_5") != std::string::npos);
        arm_safety = saved_guard;
        EtherCAT_Command_Set();
    }

    motor_position_offsets.load(YAML::Load("motorPositionOffsets: {left_shoulder_roll_deg: 180}"));
    const auto previous_right = pdo_tx[1][1];
    const auto previous_left = pdo_tx[0][0];
    EtherCAT_Command_Set();
    assert(arm_safety.fault.empty());
    const auto& shifted = pdo_tx[0][1];
    const int encoded_q = (shifted.data[3] << 8) | shifted.data[4];
    assert(std::abs(uint_to_float(encoded_q, POS_MIN, POS_MAX, 16) - M_PI) < .001);
    assert(std::memcmp(&previous_right, &pdo_tx[1][1], sizeof(MotorBlock)) == 0);
    assert(std::memcmp(&previous_left, &pdo_tx[0][0], sizeof(MotorBlock)) == 0);
    ServoStruct.CAN_boards[0].can_jnt_des_msg[1].transMod = 4;
    EtherCAT_Command_Set();
    const auto& position_packet = pdo_tx[0][1];
    const uint32_t position_bits = (uint32_t(position_packet.data[0] & 0x1f) << 27) |
        (uint32_t(position_packet.data[1]) << 19) | (uint32_t(position_packet.data[2]) << 11) |
        (uint32_t(position_packet.data[3]) << 3) | (position_packet.data[4] >> 5);
    float degrees;
    std::memcpy(&degrees, &position_bits, sizeof(degrees));
    assert(degrees == 180);
    ServoStruct.CAN_boards[0].can_jnt_des_msg[1].transMod = 2;

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
            const int q = (block.data[3] << 8) | block.data[4];
            assert(q == float_to_uint(0, POS_MIN, POS_MAX, 16));  // Damping is never offset.
            assert(tor == float_to_uint(0, T_MIN, T_MAX, 12));
        }
    }

    MotorBlock input[CAN_CHANNELS][MOTORS_PER_CH]{};
    ecat_phys = 1;
    ec_slave[1].inputs = reinterpret_cast<uint8*>(input);
    expectedWKC = wkc = 3;
    auto& offset_feedback = input[0][1];
    offset_feedback.id = 2; offset_feedback.dlc = 7; offset_feedback.data[0] = 0x20;
    const int raw_q = float_to_uint(M_PI + .2, POS_MIN, POS_MAX, 16);
    offset_feedback.data[1] = raw_q >> 8; offset_feedback.data[2] = raw_q & 255;
    offset_feedback.data[3] = 128;
    EtherCAT_Data_Get();
    assert(std::abs(arm_safety.feedback[1].position - .2) < .001);
    assert(std::abs(ServoStruct.CAN_boards[0].can_jnt_ret_msg[1].joint_posRet - .2) < .001);
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

    MotorBlock output[CAN_CHANNELS][MOTORS_PER_CH]{};
    ec_slave[1].outputs = reinterpret_cast<uint8*>(output);
    ec_slave[1].Ibytes = ec_slave[1].Obytes = ETC_PDO_SIZE;
    for (int channel : {0, 1}) {
        MotorBlock zero{};
        MotorZeroSetting(&zero, 2);
        bool valid_wkc = false;
        MotorSingleExchange(channel, 2, zero, valid_wkc);
        assert(valid_wkc);
        for (int ch = 0; ch < CAN_CHANNELS; ++ch) {
            for (int slot = 0; slot < MOTORS_PER_CH; ++slot) {
                if (ch == channel && slot == 1) {
                    assert(last_sent[ch][slot].id == 0x7ff && last_sent[ch][slot].data[1] == 2);
                    assert(last_sent[ch][slot].dlc == 4);
                } else assert(last_sent[ch][slot].dlc == 0);
                assert(output[ch][slot].dlc == 0);  // No zero command remains queued.
            }
        }
    }
    for (int parameter = 1; parameter <= 3; ++parameter) {
        MotorFeedbackQueryStep(false);
        for (int ch = 0; ch < 2; ++ch) {
            for (int j = 0; j < 8; ++j) {
                assert(output[ch][j].id == j + 1);
                assert(output[ch][j].dlc == 2);
                assert(output[ch][j].data[0] == 0xe0);
                assert(output[ch][j].data[1] == parameter);
            }
        }
    }
    // Query replies never alter the live controller's feedback or fault state.
    assert(arm_safety.feedback[0].received == 123);
    assert(!arm_safety.fault.empty());
    MotorFeedbackQuery left, right;
    MotorBlock reply{};
    reply.id = 2; reply.dlc = 6; reply.data[0] = 0xa1; reply.data[1] = 1;
    const float query_degrees = -12.5;
    uint32_t bits;
    std::memcpy(&bits, &query_degrees, sizeof(bits));
    for (int i = 0; i < 4; ++i) reply.data[2 + i] = bits >> (24 - 8 * i);
    assert(left.accept(reply, 2, true, 10));
    assert(left.value[0] == query_degrees && left.error[0] == 1);
    assert(left.fresh(0, 10.1) && !left.fresh(0, 10.51));
    assert(!right.fresh(0, 10));
    assert(!left.accept(reply, 1, true, 11));
    assert(!left.accept(reply, 2, false, 11));
    reply.dlc = 5;
    assert(!left.accept(reply, 2, true, 11));
    reply.dlc = 6; reply.data[0] = 0x20;
    assert(!left.accept(reply, 2, true, 11));
    assert(left.received[0] == 10);
}
