#include "servo_control/motor_control.h"

union RV_TypeConvert
{
    float to_float;
    int to_int;
    unsigned int to_uint;
    uint8_t buf[4];
} rv_type_convert;

union RV_TypeConvert2
{
    int16_t to_int16;
    uint16_t to_uint16;
    uint8_t buf[2];
} rv_type_convert2;

MotorCommFbd motor_comm_fbd;
OD_Motor_Msg rv_motor_msg[16];
fdback m_fdback[7];

// MOTOR SETTING
/*
cmd:
0x00:NON
0x01:set the communication mode to automatic feedback.
0x02:set the communication mode to response.
0x03:set the current position to zero.
*/
void MotorSetting(MotorBlock *block, uint16_t motor_id, uint8_t cmd)
{
    // EtherCAT_Msg TxMessage;

    block->id = 0x7FF;
    block->frame = MOTOR_FRAME;
    block->dlc = 4;

    if (cmd == 0)
        return;

    block->data[0] = motor_id >> 8;
    block->data[1] = motor_id & 0xff;
    block->data[2] = 0x00;
    block->data[3] = cmd;
}

// 专门的零点设置函数
void MotorZeroSetting(MotorBlock *block, uint16_t motor_id)
{
    block->id = 0x7FF;
    block->frame = MOTOR_FRAME;
    block->dlc = 4;

    block->data[0] = motor_id >> 8;
    block->data[1] = motor_id & 0xff;
    block->data[2] = 0x00;
    block->data[3] = 0x03;  // 零点设置命令
    
    printf("[Zero Set] Sending zero command: motor_id=%d\n", motor_id);
}

// Reset Motor ID
void MotorIDReset(MotorBlock *block)
{
    block->id = 0x7FF;
    block->dlc = 6;
    block->frame = MOTOR_FRAME;

    block->data[0] = 0x7F;
    block->data[1] = 0x7F;
    block->data[2] = 0x00;
    block->data[3] = 0x05;
    block->data[4] = 0x7F;
    block->data[5] = 0x7F;
}
// set motor new ID
void MotorIDSetting(MotorBlock *block, uint16_t motor_id, uint16_t motor_id_new)
{
    block->id = 0x7FF;
    block->dlc = 6;
    block->frame = MOTOR_FRAME;

    block->data[0] = motor_id >> 8;
    block->data[1] = motor_id & 0xff;
    block->data[2] = 0x00;
    block->data[3] = 0x04;
    block->data[4] = motor_id_new >> 8;
    block->data[5] = motor_id_new & 0xff;
}
// read motor communication mode
void MotorCommModeReading(MotorBlock *block, uint16_t motor_id)
{
    block->frame = MOTOR_FRAME;
    block->id = 0x7FF;
    block->dlc = 4;

    block->data[0] = motor_id >> 8;
    block->data[1] = motor_id & 0xff;
    block->data[2] = 0x00;
    block->data[3] = 0x81;
}
// read motor ID
void MotorIDReading(MotorBlock *block)
{
    block->frame = MOTOR_FRAME;
    block->id = 0x7FF;
    block->dlc = 4;

    block->data[0] = 0xFF;
    block->data[1] = 0xFF;
    block->data[2] = 0x00;
    block->data[3] = 0x82;
}

// This function use in ask communication mode.
/*
motor_id:1~0x7FE
kp:0~500
kd:0~50
pos:-12.5rad~12.5rad
spd:-18rad/s~18rad/s
tor:-30Nm~30Nm
*/
void send_motor_ctrl_cmd(MotorBlock *block, uint16_t motor_id, float kp, float kd, float pos, float spd, float tor)
{
    int kp_int;
    int kd_int;
    int pos_int;
    int spd_int;
    int tor_int;
    //printf("test call back : pos=%.5f, spd=%.5f, tor=%.5f \n", pos, spd, tor);


    block->frame = MOTOR_FRAME;
    block->id = motor_id;
    block->dlc = 8;

    if (kp > KP_MAX)
        kp = KP_MAX;
    else if (kp < KP_MIN)
        kp = KP_MIN;
    if (kd > KD_MAX)
        kd = KD_MAX;
    else if (kd < KD_MIN)
        kd = KD_MIN;
    if (pos > POS_MAX)
        pos = POS_MAX;
    else if (pos < POS_MIN)
        pos = POS_MIN;
    if (spd > SPD_MAX)
        spd = SPD_MAX;
    else if (spd < SPD_MIN)
        spd = SPD_MIN;
    if (tor > T_MAX)
        tor = T_MAX;
    else if (tor < T_MIN)
        tor = T_MIN;

    kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 9);
    pos_int = float_to_uint(pos, POS_MIN, POS_MAX, 16);
    spd_int = float_to_uint(spd, SPD_MIN, SPD_MAX, 12);
    tor_int = float_to_uint(tor, T_MIN, T_MAX, 12);

    block->data[0] = 0x00 | (kp_int >> 7);                             // kp5
    block->data[1] = ((kp_int & 0x7F) << 1) | ((kd_int & 0x100) >> 8); // kp7+kd1
    block->data[2] = kd_int & 0xFF;
    block->data[3] = pos_int >> 8;
    block->data[4] = pos_int & 0xFF;
    block->data[5] = spd_int >> 4;
    block->data[6] = (spd_int & 0x0F) << 4 | (tor_int >> 8);
    block->data[7] = tor_int & 0xff;
}

// This function use in ask communication mode.
/*
motor_id:1~0x7FE
pos:float
spd:0~18000
cur:0~3000
ack_status:0~3
*/
void set_motor_position(MotorBlock *block, uint16_t motor_id, float pos, uint16_t spd, uint16_t cur, uint8_t ack_status)
{

    block->frame = MOTOR_FRAME;
    block->id = motor_id;
    block->dlc = 8;

    if (ack_status > 3)
        return;

    rv_type_convert.to_float = pos;
    block->data[0] = 0x20 | (rv_type_convert.buf[3] >> 3);
    block->data[1] = (rv_type_convert.buf[3] << 5) | (rv_type_convert.buf[2] >> 3);
    block->data[2] = (rv_type_convert.buf[2] << 5) | (rv_type_convert.buf[1] >> 3);
    block->data[3] = (rv_type_convert.buf[1] << 5) | (rv_type_convert.buf[0] >> 3);
    block->data[4] = (rv_type_convert.buf[0] << 5) | (spd >> 10);
    block->data[5] = (spd & 0x3FC) >> 2;
    block->data[6] = (spd & 0x03) << 6 | (cur >> 6);
    block->data[7] = (cur & 0x3F) << 2 | ack_status;
}

// This function use in ask communication mode.
/*
motor_id:1~0x7FE
spd:-18000~18000
cur:0~3000
ack_status:0~3
*/
void set_motor_speed(MotorBlock *block, uint16_t motor_id, float spd, uint16_t cur, uint8_t ack_status)
{

    block->frame = MOTOR_FRAME;
    block->id = motor_id;
    block->dlc = 7;

    rv_type_convert.to_float = spd;
    block->data[0] = 0x40 | ack_status;
    block->data[1] = rv_type_convert.buf[3];
    block->data[2] = rv_type_convert.buf[2];
    block->data[3] = rv_type_convert.buf[1];
    block->data[4] = rv_type_convert.buf[0];
    block->data[5] = cur >> 8;
    block->data[6] = cur & 0xff;
}

// This function use in ask communication mode.
/*
motor_id:1~0x7FE
cur:-3000~3000
ctrl_status:
    0:current control
    1:torque control
    2:variable damping brake control(also call full brake)
    3:dynamic brake control
    4:regenerative brake control
    5:NON
    6:NON
    7:NON
ack_status:0~3 -- 返回值类型
*/
void set_motor_cur_tor(MotorBlock *block, uint16_t motor_id, int16_t cur_tor, uint8_t ctrl_status, uint8_t ack_status)
{

    block->frame = MOTOR_FRAME;
    block->id = motor_id;
    block->dlc = 3;

    if (ack_status > 3)
        return;
    if (ctrl_status > 7)
        return;
    if (ctrl_status) // enter torque control mode or brake mode
    {
        if (cur_tor > 3000)
            cur_tor = 3000;
        else if (cur_tor < -3000)
            cur_tor = -3000;
    }
    else
    {
        if (cur_tor > 2000)
            cur_tor = 2000;
        else if (cur_tor < -2000)
            cur_tor = -2000;
    }

    block->data[0] = 0x60 | ctrl_status << 2 | ack_status;
    block->data[1] = cur_tor >> 8;
    block->data[2] = cur_tor & 0xff;
}

// This function use in ask communication mode.
/*
motor_id:1~0x7FE
acc:0~2000
ack_status:0~3
*/
void set_motor_acceleration(MotorBlock *block, uint16_t motor_id, uint16_t acc, uint8_t ack_status)
{


    block->frame = MOTOR_FRAME;
    block->id = motor_id;
    block->dlc = 4;

    if (ack_status > 2)
        return;
    if (acc > 2000)
        acc = 2000;

    block->data[0] = 0xC0 | ack_status;
    block->data[1] = 0x01;
    block->data[2] = acc >> 8;
    block->data[3] = acc & 0xff;
}

// This function use in ask communication mode.
/*
motor_id:1~0x7FE
linkage:0~10000
speedKI:0~10000
ack_status:0/1
*/
void set_motor_linkage_speedKI(MotorBlock *block, uint16_t motor_id, uint16_t linkage, uint16_t speedKI, uint8_t ack_status)
{

    block->frame = MOTOR_FRAME;
    block->id = motor_id;
    block->dlc = 6;

    if (ack_status > 2)
        return;
    if (linkage > 10000)
        linkage = 10000;
    if (speedKI > 10000)
        speedKI = 10000;

    block->data[0] = 0xC0 | ack_status;
    block->data[1] = 0x02;
    block->data[2] = linkage >> 8;
    block->data[3] = linkage & 0xff;
    block->data[4] = speedKI >> 8;
    block->data[5] = speedKI & 0xff;
}

// This function use in ask communication mode.
/*
motor_id:1~0x7FE
fdbKP:0~10000
fbdKD:0~10000
ack_status:0/1
*/
void set_motor_posLoop_KP_KD(MotorBlock *block, uint16_t motor_id, uint16_t fdbKP, uint16_t fdbKD, uint8_t ack_status)
{

    block->frame = MOTOR_FRAME;
    block->id = motor_id;
    block->dlc = 6;

    if (ack_status > 2)
        return;
    if (fdbKP > 10000)
        fdbKP = 10000;
    if (fdbKD > 10000)
        fdbKD = 10000;

    block->data[0] = 0xC0 | ack_status;
    block->data[1] = 0x0e;
    block->data[2] = fdbKP >> 8;
    block->data[3] = fdbKP & 0xff;
    block->data[4] = fdbKD >> 8;
    block->data[5] = fdbKD & 0xff;
}

void set_motor_spdLoop_KP_KI(MotorBlock *block, uint16_t motor_id, uint16_t speedKp, uint16_t speedKI, uint8_t ack_status){
    block->frame = MOTOR_FRAME;
    block->id = motor_id;
    block->dlc = 6;

    if (ack_status > 2)
        return;
    if (speedKp > 10000)
        speedKp = 10000;
    if (speedKI > 10000)
        speedKI = 10000;

    block->data[0] = 0xC0 | ack_status;
    block->data[1] = 0x0d;
    block->data[2] = speedKp >> 8;
    block->data[3] = speedKp & 0xff;
    block->data[4] = speedKI >> 8;
    block->data[5] = speedKI & 0xff;
}

// This function use in ask communication mode.
/*
motor_id:1~0x7FE
param_cmd:1~9
*/
void get_motor_parameter(MotorBlock *block, uint16_t motor_id, uint8_t param_cmd)
{


    block->frame = MOTOR_FRAME;
    block->id = motor_id;
    block->dlc = 2;

    block->data[0] = 0xE0;
    block->data[1] = param_cmd;
}

extern fdback m_fdback[7];
void Rv_Message_Print(uint8_t ack_status, int slave)
{
    if (ack_status >= 100)
    {
        if(motor_comm_fbd.motor_fbd > 0){
            printf("motor%d message:\n", ack_status - 100);
        }
        //printf("ctl_word = %d \n", motor_comm_fbd.motor_fbd);
        if (motor_comm_fbd.motor_fbd == 0x01)
        {
            printf("自动模式.\n");
        }
        else if (motor_comm_fbd.motor_fbd == 0x02)
        {
            printf("问答模式.\n");
        }
        else if (motor_comm_fbd.motor_fbd == 0x03)
        {
            printf("零点设置成功.\n");
        }
        else if (motor_comm_fbd.motor_fbd == 0x04)
        {
            printf("新设置的Id为: %d\n", motor_comm_fbd.motor_id);
        }
        else if (motor_comm_fbd.motor_fbd == 0x05)
        {
            printf("重置Id成功.\n");
        }
        else if (motor_comm_fbd.motor_fbd == 0x06)
        {
            printf("当前电机Id: %d\n", motor_comm_fbd.motor_id);
        }
        else if (motor_comm_fbd.motor_fbd == 0x80)
        {
            printf("查询失败.\n");
        }
    }
    else
    {
        int motor_count = 0;  // 记录有效电机数量
        for (int i = 0; i < 16; ++i)
        {
            if (rv_motor_msg[i].motor_id == 0)
            {
                continue;
            }
            motor_count++;
            
            // 添加分隔符，区分不同电机
            if (motor_count > 1) {
                printf("---\n");
            }
            
            switch (ack_status)
            {
            case 1:
                printf("motor id: %d\n", rv_motor_msg[i].motor_id);
                printf("angle_actual_rad: %f\n", rv_motor_msg[i].angle_actual_rad);
                printf("speed_actual_rad: %f\n", rv_motor_msg[i].speed_actual_rad);
                printf("current_actual_float: %f\n", rv_motor_msg[i].current_actual_float);
                printf("temperature: %d\n", rv_motor_msg[i].temperature);

                //返回值存入全局变量
                
                break;
            case 2:
                printf("motor id: %d\n", rv_motor_msg[i].motor_id);
                printf("angle_actual_float: %f\n", rv_motor_msg[i].angle_actual_float);
                printf("current_actual_int: %d\n", rv_motor_msg[i].current_actual_int);
                printf("temperature: %d\n", rv_motor_msg[i].temperature);
                printf("current_actual_float: %f\n", rv_motor_msg[i].current_actual_float);

                //返回值存入结构体并输出
                //暂时用不到，因此不存
                break;
            case 3:
                printf("motor id: %d\n", rv_motor_msg[i].motor_id);
                printf("speed_actual_float: %f\n", rv_motor_msg[i].speed_actual_float);
                printf("current_actual_int: %d\n", rv_motor_msg[i].current_actual_int);
                printf("temperature: %d\n", rv_motor_msg[i].temperature);
                printf("current_actual_float: %f\n", rv_motor_msg[i].current_actual_float);
                break;
            case 4:
                if (motor_comm_fbd.motor_fbd == 0)
                {
                    printf("配置成功.\n");
                }
                else
                {
                    printf("配置失败.\n");
                }
                break;
            case 5:
                printf("motor id: %d\n", rv_motor_msg[i].motor_id);
                if (motor_comm_fbd.INS_code == 1)
                {
                    printf("angle_actual_float: %f\n", rv_motor_msg[i].angle_actual_float);
                }
                else if (motor_comm_fbd.INS_code == 2)
                {
                    printf("speed_actual_float: %f\n", rv_motor_msg[i].speed_actual_float);
                }
                else if (motor_comm_fbd.INS_code == 3)
                {
                    printf("current_actual_float: %f\n", rv_motor_msg[i].current_actual_float);
                }
                else if (motor_comm_fbd.INS_code == 4)
                {
                    printf("power: %f\n", rv_motor_msg[i].power);
                }
                else if (motor_comm_fbd.INS_code == 5)
                {
                    printf("acceleration: %d\n", rv_motor_msg[i].acceleration);
                }
                else if (motor_comm_fbd.INS_code == 6)
                {
                    printf("linkage_KP: %d\n", rv_motor_msg[i].linkage_KP);
                }
                else if (motor_comm_fbd.INS_code == 7)
                {
                    printf("speed_KI: %d\n", rv_motor_msg[i].speed_KI);
                }
                else if (motor_comm_fbd.INS_code == 8)
                {
                    printf("feedback_KP: %d\n", rv_motor_msg[i].feedback_KP);
                }
                else if (motor_comm_fbd.INS_code == 9)
                {
                    printf("feedback_KD: %d\n", rv_motor_msg[i].feedback_KD);
                }
                break;
            case 6:
                printf("motor id: %d\n", rv_motor_msg[i].motor_id);
                printf("angle_actual_int: %d\n", rv_motor_msg[i].angle_actual_int);
                printf("speed_actual_int: %d\n", rv_motor_msg[i].speed_actual_int);
                printf("current_actual_int: %d\n", rv_motor_msg[i].current_actual_int);
                printf("temperature: %d\n", rv_motor_msg[i].temperature);
                break;
            default:
                break;
            }
        }
        
        // 如果没有找到任何有效电机，打印提示
        if (motor_count == 0) {
            printf("No valid motor data found.\n");
        }
    }
}

uint16_t motor_id_check = 0;
uint8_t RV_can_data_repack(MotorBlock *rx_block, uint8_t comm_mode, uint8_t slave_id)
{
    uint8_t motor_id_t = 0;
    uint8_t ack_status = 0;
    uint8_t processed_count = 0;  // 新增：记录处理的电机数量
    int pos_int = 0;
    int spd_int = 0;
    int cur_int = 0;

    if (rx_block->dlc == 0)
        return 255;

    if (rx_block->id == 0x7FF)
    {
        if (rx_block->data[2] != 0x01)
            return 255;
        if ((rx_block->data[0] == 0xff) && (rx_block->data[1] == 0xFF))
        {
            motor_comm_fbd.motor_id = rx_block->data[3] << 8 | rx_block->data[4];
            motor_comm_fbd.motor_fbd = 0x06;
        }
        else if ((rx_block->data[0] == 0x80) && (rx_block->data[1] == 0x80))
        {
            motor_comm_fbd.motor_id = 0;
            motor_comm_fbd.motor_fbd = 0x80;
        }
        else if ((rx_block->data[0] == 0x7F) && (rx_block->data[1] == 0x7F))
        {
            motor_comm_fbd.motor_id = 1;
            motor_comm_fbd.motor_fbd = 0x05;
        }
        else
        {
            motor_comm_fbd.motor_id = rx_block->data[0] << 8 | rx_block->data[1];
            motor_comm_fbd.motor_fbd = rx_block->data[3];
            if (rx_block->data[3] == 0x03) {
                printf("[Zero Set] Motor ID %d zero setting successful!\n", motor_comm_fbd.motor_id);
            }
        }
        return 100;
    }
    else if (comm_mode == 0x00 && rx_block->dlc != 0)
    {
            //printf("==============================can data get success==============================\n");
            // printf("id = %d\n"i);
            ack_status = rx_block->data[0] >> 5;
            motor_id_t = rx_block->id - 1;
            motor_id_check = rx_block->id;
            rv_motor_msg[motor_id_t].motor_id = motor_id_check;
            rv_motor_msg[motor_id_t].error = rx_block->data[0] & 0x1F;
            
            //printf("DEBUG: Processing motor_id=%d, motor_id_t=%d, ack_status=%d\n", motor_id_check, motor_id_t, ack_status);
            
            if (ack_status == 1) // response frame 1
            {
                pos_int = rx_block->data[1] << 8 | rx_block->data[2];
                spd_int = rx_block->data[3] << 4 | (rx_block->data[4] & 0xF0) >> 4;
                cur_int = (rx_block->data[4] & 0x0F) << 8 | rx_block->data[5];


                rv_motor_msg[motor_id_t].angle_actual_rad = uint_to_float(pos_int, POS_MIN, POS_MAX, 16);
                rv_motor_msg[motor_id_t].speed_actual_rad = uint_to_float(spd_int, SPD_MIN, SPD_MAX, 12);
                rv_motor_msg[motor_id_t].current_actual_float = uint_to_float(cur_int, I_MIN, I_MAX, 12);
                rv_motor_msg[motor_id_t].temperature = (rx_block->data[6] - 50) / 2;

                //printf("mid = %i \n", motor_id_t);
                //printf("pos int = %i \n", pos_int);
                //printf("pos float = %f \n", rv_motor_msg[motor_id_t].angle_actual_rad);
            }
            else if (ack_status == 2) // response frame 2
            {
                rv_type_convert.buf[0] = rx_block->data[4];
                rv_type_convert.buf[1] = rx_block->data[3];
                rv_type_convert.buf[2] = rx_block->data[2];
                rv_type_convert.buf[3] = rx_block->data[1];
                rv_motor_msg[motor_id_t].angle_actual_float = rv_type_convert.to_float;
                rv_motor_msg[motor_id_t].current_actual_int = rx_block->data[5] << 8 | rx_block->data[6];
                rv_motor_msg[motor_id_t].temperature = (rx_block->data[7] - 50) / 2;
                rv_motor_msg[motor_id_t].current_actual_float = rv_motor_msg[motor_id_t].current_actual_int / 100.0f;
            }
            else if (ack_status == 3) // response frame 3
            {
                rv_type_convert.buf[0] = rx_block->data[4];
                rv_type_convert.buf[1] = rx_block->data[3];
                rv_type_convert.buf[2] = rx_block->data[2];
                rv_type_convert.buf[3] = rx_block->data[1];
                rv_motor_msg[motor_id_t].speed_actual_float = rv_type_convert.to_float;
                rv_motor_msg[motor_id_t].current_actual_int = rx_block->data[5] << 8 | rx_block->data[6];
                rv_motor_msg[motor_id_t].temperature = (rx_block->data[7] - 50) / 2;
                rv_motor_msg[motor_id_t].current_actual_float = rv_motor_msg[motor_id_t].current_actual_int / 100.0f;
            }
            else if (ack_status == 4) // response frame 4
            {
                if (rx_block->dlc != 3)
                    return 255;
                motor_comm_fbd.INS_code = rx_block->data[1];
                motor_comm_fbd.motor_fbd = rx_block->data[2];
            }
            else if (ack_status == 5) // response frame 5
            {
                motor_comm_fbd.INS_code = rx_block->data[1];
                if (motor_comm_fbd.INS_code == 1 && rx_block->dlc == 6) // get position
                {
                    rv_type_convert.buf[0] = rx_block->data[5];
                    rv_type_convert.buf[1] = rx_block->data[4];
                    rv_type_convert.buf[2] = rx_block->data[3];
                    rv_type_convert.buf[3] = rx_block->data[2];
                    rv_motor_msg[motor_id_t].angle_actual_float = rv_type_convert.to_float;
                }
                else if (motor_comm_fbd.INS_code == 2 && rx_block->dlc == 6) // get speed
                {
                    rv_type_convert.buf[0] = rx_block->data[5];
                    rv_type_convert.buf[1] = rx_block->data[4];
                    rv_type_convert.buf[2] = rx_block->data[3];
                    rv_type_convert.buf[3] = rx_block->data[2];
                    rv_motor_msg[motor_id_t].speed_actual_float = rv_type_convert.to_float;
                }
                else if (motor_comm_fbd.INS_code == 3 && rx_block->dlc == 6) // get current
                {
                    rv_type_convert.buf[0] = rx_block->data[5];
                    rv_type_convert.buf[1] = rx_block->data[4];
                    rv_type_convert.buf[2] = rx_block->data[3];
                    rv_type_convert.buf[3] = rx_block->data[2];
                    rv_motor_msg[motor_id_t].current_actual_float = rv_type_convert.to_float;
                }
                else if (motor_comm_fbd.INS_code == 4 && rx_block->dlc == 6) // get power
                {
                    rv_type_convert.buf[0] = rx_block->data[5];
                    rv_type_convert.buf[1] = rx_block->data[4];
                    rv_type_convert.buf[2] = rx_block->data[3];
                    rv_type_convert.buf[3] = rx_block->data[2];
                    rv_motor_msg[motor_id_t].power = rv_type_convert.to_float;
                }
                else if (motor_comm_fbd.INS_code == 5 && rx_block->dlc == 4) // get acceleration
                {
                    rv_motor_msg[motor_id_t].acceleration = rx_block->data[2] << 8 | rx_block->data[3];
                }
                else if (motor_comm_fbd.INS_code == 6 && rx_block->dlc == 4) // get linkage_KP
                {
                    rv_motor_msg[motor_id_t].linkage_KP = rx_block->data[2] << 8 | rx_block->data[3];
                }
                else if (motor_comm_fbd.INS_code == 7 && rx_block->dlc == 4) // get speed_KI
                {
                    rv_motor_msg[motor_id_t].speed_KI = rx_block->data[2] << 8 | rx_block->data[3];
                }
                else if (motor_comm_fbd.INS_code == 8 && rx_block->dlc == 4) // get feedback_KP
                {
                    rv_motor_msg[motor_id_t].feedback_KP = rx_block->data[2] << 8 | rx_block->data[3];
                }
                else if (motor_comm_fbd.INS_code == 9 && rx_block->dlc == 4) // get feedback_KD
                {
                    rv_motor_msg[motor_id_t].feedback_KD = rx_block->data[2] << 8 | rx_block->data[3];
                }
            }

            processed_count++;  // 记录已处理的电机数量
            //printf("=========================================================================================\n");
            //printf("DEBUG: Processed motor %d, total processed: %d\n", motor_id_check, processed_count);
            // 不要立即return，继续处理下一个通道
        }
        else if (comm_mode == 0x01 && rx_block->dlc != 0) // automatic feedback mode
        {
            motor_id_t = rx_block->id - 0x205;
            rv_motor_msg[motor_id_t].motor_id = rx_block->id;
            rv_motor_msg[motor_id_t].angle_actual_int = (uint16_t)(rx_block->data[0] << 8 | rx_block->data[1]);
            rv_motor_msg[motor_id_t].speed_actual_int = (int16_t)(rx_block->data[2] << 8 | rx_block->data[3]);
            rv_motor_msg[motor_id_t].current_actual_int = (rx_block->data[4] << 8 | rx_block->data[5]);
            rv_motor_msg[motor_id_t].temperature = rx_block->data[6];
            rv_motor_msg[motor_id_t].error = rx_block->data[7];
            return 6;
        }

    return processed_count > 0 ? ack_status : 255;
}
