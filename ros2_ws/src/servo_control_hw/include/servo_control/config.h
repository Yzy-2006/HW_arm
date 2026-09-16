/*
 * @Description: ECAT-8CAN Gateway PDO definitions
 * @Author: kx zhang
 * @Date: 2022-09-20 09:21:54
 * @LastEditTime: 2022-11-13 17:00:10
 */
#pragma once

#include <inttypes.h>
#include <string.h>

//********************************************//
//***********ECAT-8CAN PDO Layout*************//
//********************************************//
// Per protocol.md §4:
//   3 CAN channels × 8 motor slots × 14-byte MotorBlock = 336 bytes PDO

#define CAN_CHANNELS      3
#define MOTORS_PER_CH     8
#define MOTOR_BLOCK_SIZE  14
#define CHANNEL_PDO_SIZE  (MOTORS_PER_CH * MOTOR_BLOCK_SIZE)   // 112
#define ETC_PDO_SIZE (CAN_CHANNELS * CHANNEL_PDO_SIZE)    // 336

#pragma pack(push, 1)

typedef struct
{
    uint32_t id;        // CAN message ID (11-bit standard or 29-bit extended)
    uint8_t  frame;     // bit0=EXT, bit1=CANFD, bit2=BRS, bit3=RTR, bit4-7=0
    uint8_t  dlc;       // Data Length Code (0-8)
    uint8_t  data[8];   // Data payload
} MotorBlock;           // 14 bytes per protocol.md §4.2

#pragma pack(pop)
