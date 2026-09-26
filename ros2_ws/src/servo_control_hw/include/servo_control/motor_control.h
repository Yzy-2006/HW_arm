/*
 * @Description: Motor CAN control protocol (ECAT-8CAN gateway)
 * @Author: kx zhang
 * @Date: 2022-09-20 09:45:09
 * @LastEditTime: 2022-11-13 17:16:19
 */
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <inttypes.h>
#include <string.h>
#include "servo_control/config.h"
#include "servo_control/math_ops.h"

#define param_get_pos 0x01
#define param_get_spd 0x02
#define param_get_cur 0x03
#define param_get_pwr 0x04
#define param_get_acc 0x05
#define param_get_lkgKP 0x06
#define param_get_spdKI 0x07
#define param_get_fdbKP 0x08
#define param_get_fdbKD 0x09

#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 50.0f
#define POS_MIN -12.5f
#define POS_MAX 12.5f
#define SPD_MIN -18.0f
#define SPD_MAX 18.0f
#define T_MIN -90.0f
#define T_MAX 90.0f
#define I_MIN -90.0f
#define I_MAX 90.0f

#define comm_ack 0x00
#define comm_auto 0x01

// FRAME byte: bit0=EXT, bit1=CANFD, bit2=BRS, bit3=RTR
// 0x00=Classic CAN, 0x06=CANFD+BRS (per protocol.md §4.3) 0x06 测试上位机收不到指令
#define MOTOR_FRAME 0x06

#define SLAVE_NUMBER 12 //最大从机数

typedef struct
{
    uint16_t motor_id;
    uint8_t INS_code;  // instruction code.
    uint8_t motor_fbd; // motor CAN communication feedback.
} MotorCommFbd;

typedef struct
{
    uint16_t angle_actual_int;
    uint16_t angle_desired_int;
    int16_t speed_actual_int;
    int16_t speed_desired_int;
    int16_t current_actual_int;
    int16_t current_desired_int;
    float speed_actual_rad;
    float speed_desired_rad;
    float angle_actual_rad;
    float angle_desired_rad;
    uint16_t motor_id;
    int8_t temperature; // Decoded Celsius can be negative.
    uint8_t error;
    float angle_actual_float;
    float speed_actual_float;
    float current_actual_float;
    float angle_desired_float;
    float speed_desired_float;
    float current_desired_float;
    float power;
    uint16_t acceleration;
    uint16_t linkage_KP;
    uint16_t speed_KI;
    uint16_t feedback_KP;
    uint16_t feedback_KD;
} OD_Motor_Msg;

typedef struct
{
    uint8_t JntID;
    uint8_t type;
    uint8_t error_type;
    float pos;
    float vel;
    float Iq;
    float temperature_Motor;
    uint8_t config_code;
    uint8_t config_state;
}fdback;

extern OD_Motor_Msg rv_motor_msg[16];
extern uint16_t motor_id_check;
extern fdback m_fdback[7];

// Broadcast commands (CAN ID 0x7FF) — write to a specific motor block
void MotorIDReset(MotorBlock *block);
void MotorIDSetting(MotorBlock *block, uint16_t motor_id, uint16_t motor_id_new);
void MotorSetting(MotorBlock *block, uint16_t motor_id, uint8_t cmd);
void MotorZeroSetting(MotorBlock *block, uint16_t motor_id);
void MotorCommModeReading(MotorBlock *block, uint16_t motor_id);
void MotorIDReading(MotorBlock *block);

// Motor control commands — write to a specific motor block
void send_motor_ctrl_cmd(MotorBlock *block, uint16_t motor_id, float kp, float kd, float pos, float spd, float cur);
void set_motor_position(MotorBlock *block, uint16_t motor_id, float pos, uint16_t spd, uint16_t cur, uint8_t ack_status);
void set_motor_speed(MotorBlock *block, uint16_t motor_id, float spd, uint16_t cur, uint8_t ack_status);
void set_motor_cur_tor(MotorBlock *block, uint16_t motor_id, int16_t cur_tor, uint8_t ctrl_status, uint8_t ack_status);
void set_motor_acceleration(MotorBlock *block, uint16_t motor_id, uint16_t acc, uint8_t ack_status);
void set_motor_linkage_speedKI(MotorBlock *block, uint16_t motor_id, uint16_t linkage, uint16_t speedKI, uint8_t ack_status);
void set_motor_posLoop_KP_KD(MotorBlock *block, uint16_t motor_id, uint16_t fdbKP, uint16_t fdbKD, uint8_t ack_status);
void set_motor_spdLoop_KP_KI(MotorBlock *block, uint16_t motor_id, uint16_t speedKp, uint16_t speedKI, uint8_t ack_status);
void get_motor_parameter(MotorBlock *block, uint16_t motor_id, uint8_t param_cmd);

uint8_t RV_can_data_repack(MotorBlock *rx_block, uint8_t comm_mode, uint8_t slave_id);
void Rv_Message_Print(uint8_t ack_status, int slave);

int float_to_uint(float x, float x_min, float x_max, int bits);

#ifdef __cplusplus
}
#endif

#endif
