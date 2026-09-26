#include "ethercat.h"

#include "servo_control/motor_control.h"
#include "servo_control/transmit.h"
#include "servo_control/queue.h"

#include <sys/time.h>
#include <cinttypes>
#include <cstdio>
#include <cstring>
#include <vector>
#include "time.h"
#include <iostream>


#define EC_TIMEOUTM


// ETC gateway slave index (唯一从站)
int ecat_phys = -1;
const uint32_t ETC_VENDOR_ID = 0x00000009;
const uint32_t ETC_PRODUCT_CODE = 0x00820013;

spsc_queue<MotorBlock_ptr, capacity<30>> messages[SLAVE_NUMBER];
std::atomic<bool> running{false};
std::thread runThread;

char IOmap[4096];
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;
uint64_t num;
bool isConfig[SLAVE_NUMBER]{false};

pthread_mutex_t data_mutex = PTHREAD_MUTEX_INITIALIZER;
// 缓存上一次的有效数据
SlaveChannel_fdBack last_fbBack_Message[SLAVE_NUMBER];
bool last_data_valid = false;

#define EC_TIMEOUTMON 500

servo_struct ServoStruct;
CAN_JNT_numStruct CAN_and_Jnt_num;

static int wkc_err_count = 0;
static int wkc_err_iteration_count = 0;

void EtherCAT_Data_Get();

void EtherCAT_Command_Set();

static void degraded_handler()
{
    //printf("[EtherCAT Error] Logging error...\n");
    time_t current_time = time(NULL);
    char *time_str = ctime(&current_time);
    //printf("ESTOP. EtherCAT became degraded at %s.\n", time_str);
    //printf("[EtherCAT Error] Stopping RT process.\n");
}

static int run_ethercat(const char *ifname)
{
    int i;
    int oloop, iloop, chk;
    needlf = FALSE;
    inOP = FALSE;

    num = 1;

    /* initialise SOEM, bind socket to ifname */
    if (ec_init(ifname))
    {
        printf("[EtherCAT Init] Initialization on device %s succeeded.\n", ifname);
        /* find and auto-config slaves */

        if (ec_config_init(FALSE) > 0)
        {
            printf("[EtherCAT Init] %d slaves found and configured.\n", ec_slavecount);
            if (ec_slavecount < SLAVE_NUMBER)
            {
                printf("[RT EtherCAT] Warning: Expected %d slaves, found %d.\n", SLAVE_NUMBER, ec_slavecount);
            }

            for (int slave_idx = 0; slave_idx < ec_slavecount; slave_idx++)
                ec_slave[slave_idx+1].CoEdetails &= ~ECT_COEDET_SDOCA;

            ec_config_map(&IOmap);
            ec_configdc();

            printf("[EtherCAT Init] Mapped slaves.\n");
            /* wait for all slaves to reach SAFE_OP state */
            ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * SLAVE_NUMBER);

            for (int slave_idx = 0; slave_idx < ec_slavecount; slave_idx++)
            {
                printf("[SLAVE %d]\n", slave_idx);
                printf("  IN  %d bytes, %d bits\n", ec_slave[slave_idx].Ibytes, ec_slave[slave_idx].Ibits);
                printf("  OUT %d bytes, %d bits\n", ec_slave[slave_idx].Obytes, ec_slave[slave_idx].Obits);
                printf("\n");
            }

            oloop = ec_slave[0].Obytes;
            if ((oloop == 0) && (ec_slave[0].Obits > 0))
                oloop = 1;
            if (oloop > 8)
                oloop = 8;
            iloop = ec_slave[0].Ibytes;
            if ((iloop == 0) && (ec_slave[0].Ibits > 0))
                iloop = 1;
            if (iloop > 8)
                iloop = 8;

            printf("[EtherCAT Init] segments : %d : %d %d %d %d\n", ec_group[0].nsegments, ec_group[0].IOsegment[0], ec_group[0].IOsegment[1], ec_group[0].IOsegment[2], ec_group[0].IOsegment[3]);

            printf("[EtherCAT Init] Requesting operational state for all slaves...\n");
            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            //expectedWKC = 2;
            printf("[EtherCAT Init] Calculated workcounter %d\n", expectedWKC);
            ec_slave[0].state = EC_STATE_OPERATIONAL;
            /* send one valid process data to make outputs in slaves happy*/
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            /* request OP state for all slaves */
            ec_writestate(0);
            chk = 40;
            /* wait for all slaves to reach OP state */
            do
            {
                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);
                ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
            } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

            if (ec_slave[0].state == EC_STATE_OPERATIONAL)
            {
                printf("[EtherCAT Init] Operational state reached for all slaves.\n");
                inOP = TRUE;
                return 1;
            }
            else
            {
                printf("[EtherCAT Error] Not all slaves reached operational state.\n");
                ec_readstate();
                for (i = 1; i <= ec_slavecount; i++)
                {
                    if (ec_slave[i].state != EC_STATE_OPERATIONAL)
                    {
                        printf("[EtherCAT Error] Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                               i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    }
                }
            }
        }
        else
        {
            printf("[EtherCAT Error] No slaves found!\n");
        }
    }
    else
    {
        printf("[EtherCAT Error] No socket connection on %s - are you running run.sh?\n", ifname);
    }
    return 0;
}

static int err_count = 0;
static int err_iteration_count = 0;
/**@brief EtherCAT errors are measured over this period of loop iterations */
#define K_ETHERCAT_ERR_PERIOD 100

/**@brief Maximum number of etherCAT errors before a fault per period of loop iterations */
#define K_ETHERCAT_ERR_MAX 20

void ecatcheck()
{
    int slave = 0;

    // count errors
    if (err_iteration_count > K_ETHERCAT_ERR_PERIOD)
    {
        err_iteration_count = 0;
        err_count = 0;
    }

    if (err_count > K_ETHERCAT_ERR_MAX)
    {
        // possibly shut down
        printf("[EtherCAT Error] EtherCAT connection degraded.\n");
        printf("[Simulink-Linux] Shutting down....\n");
        degraded_handler();
        return;
    }
    err_iteration_count++;

    if (inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
    {
        if (needlf)
        {
            needlf = FALSE;
            printf("\n");
        }
        /* one ore more slaves are not responding */
        ec_group[currentgroup].docheckstate = FALSE;
        ec_readstate();
        for (slave = 1; slave <= ec_slavecount; slave++)
        {
            if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
            {
                ec_group[currentgroup].docheckstate = TRUE;
                if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                {
                    printf("[EtherCAT Error] Slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                    ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                    ec_writestate(slave);
                    err_count++;
                }
                else if (ec_slave[slave].state == EC_STATE_SAFE_OP)
                {
                    printf("[EtherCAT Error] Slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                    ec_slave[slave].state = EC_STATE_OPERATIONAL;
                    ec_writestate(slave);
                    err_count++;
                }
                else if (ec_slave[slave].state > 0)
                {
                    if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                    {
                        ec_slave[slave].islost = FALSE;
                        printf("[EtherCAT Status] Slave %d reconfigured\n", slave);
                    }
                }
                else if (!ec_slave[slave].islost)
                {
                    /* re-check state */
                    ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                    if (!ec_slave[slave].state)
                    {
                        ec_slave[slave].islost = TRUE;
                        printf("[EtherCAT Error] Slave %d lost\n", slave);
                        err_count++;
                    }
                }
            }
            if (ec_slave[slave].islost)
            {
                if (!ec_slave[slave].state)
                {
                    if (ec_recover_slave(slave, EC_TIMEOUTMON))
                    {
                        ec_slave[slave].islost = FALSE;
                        printf("[EtherCAT Status] Slave %d recovered\n", slave);
                    }
                }
                else
                {
                    ec_slave[slave].islost = FALSE;
                    printf("[EtherCAT Status] Slave %d found\n", slave);
                }
            }
        }
        if (!ec_group[currentgroup].docheckstate)
            printf("[EtherCAT Status] All slaves resumed OPERATIONAL.\n");
    }
    
}

void EtherCAT_Init(char *ifname)
{
    //开启EtherCAT通讯
    int i;
    int rc;
    printf("[EtherCAT] Initializing EtherCAT\n");
    ecatcheck();
    for (i = 1; i < 3; i++)
    {
        printf("[EtherCAT] Attempting to start EtherCAT, try %d of 3.\n", i);
        rc = run_ethercat(ifname);
        if (rc)
            break;
        osal_usleep(1000000);
    }
    if (rc)
        printf("[EtherCAT] EtherCAT successfully initialized on attempt %d \n", i);
    else
    {
        printf("[EtherCAT Error] Failed to initialize EtherCAT after 100 tries. \n");
    }
}

//----------位置控制（三环控制）参数配置函数----------

//位置环对应EtherCAT参数
MotorBlock Tx_setPosLoopPD[CAN_CHANNELS][MOTORS_PER_CH];
MotorBlock Rx_setPosLoopPD[CAN_CHANNELS][MOTORS_PER_CH];

//位置环PD参数
void EtherCAT_SetPosLoop_PD(std::vector<std::vector<double>> posKp, std::vector<std::vector<double>> posKd){
    if (wkc_err_iteration_count > K_ETHERCAT_ERR_PERIOD)
    {
        wkc_err_count = 0;
        wkc_err_iteration_count = 0;
    }
    if (wkc_err_count > K_ETHERCAT_ERR_MAX)
    {
        degraded_handler();
    }

    memset(Tx_setPosLoopPD, 0, sizeof(Tx_setPosLoopPD));
    uint8_t ask_status = 0x01;

    for(int ch = 0; ch < ServoStruct.CAN_board_num; ch++){
        int channel = ServoStruct.CAN_boards[ch].channel_idx;
        for(int j = 0; j < ServoStruct.CAN_boards[ch].JntNum_onCan; j++){
            int slot = ServoStruct.CAN_boards[ch].motor_slot[j];
            int motor_id = ServoStruct.CAN_boards[ch].can_jnt_ID[j];

            double temp_KpDes_origin = posKp[ch][j];
            double temp_KdDes_origin = posKd[ch][j];
            uint16_t temp_posKp_transmit = static_cast<uint16_t>(temp_KpDes_origin * 10000);
            uint16_t temp_posKd_transmit = static_cast<uint16_t>(temp_KdDes_origin * 10000);

            set_motor_posLoop_KP_KD(&Tx_setPosLoopPD[channel][slot], motor_id, temp_posKp_transmit, temp_posKd_transmit, ask_status);
        }
    }

    // send to ETC slave
    memcpy(ec_slave[ecat_phys].outputs, Tx_setPosLoopPD, ETC_PDO_SIZE);
    int sendRes = ec_send_processdata();

    std::this_thread::sleep_for(std::chrono::nanoseconds(50));

    auto PDInte_Feedback = Get_posLoop_PD();
}

//PD参数打印 -- 必须在有获取相关操作后使用
std::pair<std::vector<std::vector<double>>, std::vector<std::vector<double>>> Get_posLoop_PD(){
    //获取位置环PD参数

    //获取反馈值
    wkc = ec_receive_processdata(EC_TIMEOUTRET);

    //反馈值解析与打印
    std::vector<std::vector<double>> posLoop_KpGet;
    std::vector<std::vector<double>> posLoop_KdGet;
    
    // 容器大小应为机械臂板卡数
    posLoop_KpGet.resize(ServoStruct.CAN_board_num);
    posLoop_KdGet.resize(ServoStruct.CAN_board_num);

    // read from ETC slave
    if (ecat_phys > 0) {
        memcpy(Rx_setPosLoopPD, ec_slave[ecat_phys].inputs, ETC_PDO_SIZE);
    }

    for(int ch = 0; ch < ServoStruct.CAN_board_num; ch++){
        int channel = ServoStruct.CAN_boards[ch].channel_idx;
        posLoop_KpGet[ch].resize(ServoStruct.CAN_boards[ch].JntNum_onCan);
        posLoop_KdGet[ch].resize(ServoStruct.CAN_boards[ch].JntNum_onCan);

        for(int joint = 0; joint < ServoStruct.CAN_boards[ch].JntNum_onCan; joint++){
            int slot = ServoStruct.CAN_boards[ch].motor_slot[joint];
            RV_can_data_repack(&Rx_setPosLoopPD[channel][slot], comm_ack, ch);
            int rv_ID = ServoStruct.CAN_boards[ch].can_jnt_ID[joint] - 1;
            posLoop_KpGet[ch][joint] = rv_motor_msg[rv_ID].feedback_KP;
            posLoop_KdGet[ch][joint] = rv_motor_msg[rv_ID].feedback_KD;
        }
    }

    return std::make_pair(posLoop_KpGet, posLoop_KdGet);
}

//位置环对应EtherCAT参数
MotorBlock Tx_setVelLoopPI[CAN_CHANNELS][MOTORS_PER_CH];
MotorBlock Rx_setVelLoopPI[CAN_CHANNELS][MOTORS_PER_CH];

//速度环PI参数
void EtherCAT_SetVelLoop_PI(std::vector<std::vector<double>> velKp, std::vector<std::vector<double>> velKi){
    if (wkc_err_iteration_count > K_ETHERCAT_ERR_PERIOD)
    {
        wkc_err_count = 0;
        wkc_err_iteration_count = 0;
    }
    if (wkc_err_count > K_ETHERCAT_ERR_MAX)
    {
        degraded_handler();
    }

    memset(Tx_setVelLoopPI, 0, sizeof(Tx_setVelLoopPI));
    uint8_t ask_status = 0x01;

    for(int ch = 0; ch < ServoStruct.CAN_board_num; ch++){
        int channel = ServoStruct.CAN_boards[ch].channel_idx;
        for(int j = 0; j < ServoStruct.CAN_boards[ch].JntNum_onCan; j++){
            int slot = ServoStruct.CAN_boards[ch].motor_slot[j];
            int motor_id = ServoStruct.CAN_boards[ch].can_jnt_ID[j];

            uint16_t temp_velKp_transmit = static_cast<uint16_t>(velKp[ch][j] * 10000);
            uint16_t temp_velKi_transmit = static_cast<uint16_t>(velKi[ch][j] * 10000);

            set_motor_spdLoop_KP_KI(&Tx_setVelLoopPI[channel][slot], motor_id, temp_velKp_transmit, temp_velKi_transmit, ask_status);
        }
    }

    memcpy(ec_slave[ecat_phys].outputs, Tx_setVelLoopPI, ETC_PDO_SIZE);
}

//PI参数打印 -- 必须在有获取相关操作后使用
std::pair<std::vector<std::vector<double>>, std::vector<std::vector<double>>> Get_velLoop_PI(){

    // read from ETC slave
    memcpy(Rx_setVelLoopPI, ec_slave[ecat_phys].inputs, ETC_PDO_SIZE);

    std::vector<std::vector<double>> velLoop_KpGet;
    std::vector<std::vector<double>> velLoop_KiGet;
    velLoop_KpGet.resize(ServoStruct.CAN_board_num);
    velLoop_KiGet.resize(ServoStruct.CAN_board_num);

    for(int ch = 0; ch < ServoStruct.CAN_board_num; ch++){
        int channel = ServoStruct.CAN_boards[ch].channel_idx;
        velLoop_KpGet[ch].resize(ServoStruct.CAN_boards[ch].JntNum_onCan);
        velLoop_KiGet[ch].resize(ServoStruct.CAN_boards[ch].JntNum_onCan);

        for(int joint = 0; joint < ServoStruct.CAN_boards[ch].JntNum_onCan; joint++){
            int slot = ServoStruct.CAN_boards[ch].motor_slot[joint];
            RV_can_data_repack(&Rx_setVelLoopPI[channel][slot], comm_ack, ch);
            int rv_ID = ServoStruct.CAN_boards[ch].can_jnt_ID[joint] - 1;
            velLoop_KpGet[ch][joint] = rv_motor_msg[rv_ID].linkage_KP;
            velLoop_KiGet[ch][joint] = rv_motor_msg[rv_ID].speed_KI;
        }
    }

    return std::make_pair(velLoop_KpGet, velLoop_KiGet);
}

//-----------------------------------

//直接获取EtherCAT反馈值的函数
MotorBlock Tx_Msg_01[CAN_CHANNELS][MOTORS_PER_CH];
MotorBlock Rx_Msg_01[CAN_CHANNELS][MOTORS_PER_CH];

static uint32_t can_scan_cycles = 0;
static uint32_t can_scan_reply_counts[CAN_CHANNELS][MOTORS_PER_CH];

void CanChannelScanBegin()
{
    can_scan_cycles = 0;
    memset(can_scan_reply_counts, 0, sizeof(can_scan_reply_counts));
}

void CanChannelScanStep()
{
    if (ecat_phys <= 0) {
        return;
    }

    MotorBlock scan_tx[CAN_CHANNELS][MOTORS_PER_CH];
    MotorBlock scan_rx[CAN_CHANNELS][MOTORS_PER_CH];
    memset(scan_tx, 0, sizeof(scan_tx));

    for (int channel = 0; channel < CAN_CHANNELS; ++channel) {
        for (int slot = 0; slot < MOTORS_PER_CH; ++slot) {
            const int motor_id = slot + 1;
            // Current mode with a zero target is the only command emitted by
            // the scanner.  ack_status requests a reply without holding or
            // moving any joint.
            set_motor_cur_tor(&scan_tx[channel][slot], motor_id, 0, 0, 1);
        }
    }

    memcpy(ec_slave[ecat_phys].outputs, scan_tx, ETC_PDO_SIZE);
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    memcpy(scan_rx, ec_slave[ecat_phys].inputs, ETC_PDO_SIZE);
    ++can_scan_cycles;

    for (int channel = 0; channel < CAN_CHANNELS; ++channel) {
        for (int slot = 0; slot < MOTORS_PER_CH; ++slot) {
            if (scan_rx[channel][slot].dlc > 0) {
                ++can_scan_reply_counts[channel][slot];
            }
        }
    }
}

void CanChannelScanPrintResult()
{
    printf("[CAN scan] sampled %u zero-current cycles\n", can_scan_cycles);
    for (int channel = 0; channel < CAN_CHANNELS; ++channel) {
        printf("[CAN scan] CAN%d:", channel);
        for (int slot = 0; slot < MOTORS_PER_CH; ++slot) {
            printf(" ID%d=%u", slot + 1, can_scan_reply_counts[channel][slot]);
        }
        printf("\n");
    }
}

// Feedback positions stay in raw encoder coordinates, as in src-4.
static void gripperFdbDecode(int channel, int motor_id, int ack,
                             const OD_Motor_Msg& motor, const MotorBlock& frame){
    if(motor_id != 8){
        return;
    }
    auto& fdb = channel == 0 ? left_gripper_fdb : right_gripper_fdb;
    if(ack == 1){
        fdb.pos_deg = motor.angle_actual_rad * 180.0 / M_PI;
        fdb.spd_rad = motor.speed_actual_rad;
        fdb.valid = std::isfinite(fdb.pos_deg) && std::isfinite(fdb.spd_rad);
    }else if(ack == 2 || (ack == 5 && frame.dlc == 6 && frame.data[1] == 1)){
        fdb.pos_deg = motor.angle_actual_float;
        fdb.valid = std::isfinite(fdb.pos_deg);
    }
    if(ack == 1 || ack == 2 || ack == 3){
        fdb.current_A = motor.current_actual_float;
        // The decoder already converted the raw byte to Celsius.
        fdb.temperature = motor.temperature;
    }
}

void EtherCAT_direGetFdback(){
    if (wkc_err_iteration_count > K_ETHERCAT_ERR_PERIOD)
    {
        wkc_err_count = 0;
        wkc_err_iteration_count = 0;
    }
    if (wkc_err_count > K_ETHERCAT_ERR_MAX)
    {
        degraded_handler();
    }

    if (ecat_phys <= 0) return;

    memset(Tx_Msg_01, 0, sizeof(Tx_Msg_01));

    uint8_t cmd_get = 0x01;
    for(int ch = 0; ch < ServoStruct.CAN_board_num; ch++){
        int channel = ServoStruct.CAN_boards[ch].channel_idx;
        for(int j = 0; j < ServoStruct.CAN_boards[ch].JntNum_onCan; j++){
            int slot = ServoStruct.CAN_boards[ch].motor_slot[j];
            int motor_id = ServoStruct.CAN_boards[ch].can_jnt_ID[j];
            get_motor_parameter(&Tx_Msg_01[channel][slot], motor_id, cmd_get);
        }
    }

    // send to ETC slave
    memcpy(ec_slave[ecat_phys].outputs, Tx_Msg_01, ETC_PDO_SIZE);
    ec_send_processdata();

    std::this_thread::sleep_for(std::chrono::nanoseconds(50));

    // receive
    wkc = ec_receive_processdata(EC_TIMEOUTRET);
    memcpy(Rx_Msg_01, ec_slave[ecat_phys].inputs, ETC_PDO_SIZE);

    // parse feedback
    for(int ch = 0; ch < ServoStruct.CAN_board_num; ch++){
        int channel = ServoStruct.CAN_boards[ch].channel_idx;
        for(int joint = 0; joint < ServoStruct.CAN_boards[ch].JntNum_onCan; joint++){
            int slot = ServoStruct.CAN_boards[ch].motor_slot[joint];
            uint8_t repack_rc = RV_can_data_repack(&Rx_Msg_01[channel][slot], comm_ack, ch);
            if (repack_rc == 255) {
                // 无有效反馈帧：跳过，避免读到其他通道同 CAN ID 电机的残留值
                continue;
            }
            int rv_ID = ServoStruct.CAN_boards[ch].can_jnt_ID[joint] - 1;

            ServoStruct.CAN_boards[ch].can_jnt_ret_msg[joint].joint_posRet = ServoStruct.CAN_boards[ch].can_jnt_direComp[joint] * rv_motor_msg[rv_ID].angle_actual_float / 180 * M_PI;
            ServoStruct.CAN_boards[ch].can_jnt_ret_msg[joint].joint_spdRet = ServoStruct.CAN_boards[ch].can_jnt_direComp[joint] * rv_motor_msg[rv_ID].speed_actual_float / 180 * M_PI;
            ServoStruct.CAN_boards[ch].can_jnt_ret_msg[joint].joint_currentRet = rv_motor_msg[rv_ID].current_actual_float;
            ServoStruct.CAN_boards[ch].can_jnt_ret_msg[joint].joint_temperature = (rv_motor_msg[rv_ID].temperature - 50.0)/2.0;
            gripperFdbDecode(channel, ServoStruct.CAN_boards[ch].can_jnt_ID[joint],
                             repack_rc, rv_motor_msg[rv_ID], Rx_Msg_01[channel][slot]);
        }
    }
}

void EtherCAT_Transmit(MotorBlock (*MasterCommand)[MOTORS_PER_CH])
{
    memcpy(ec_slave[ecat_phys].outputs, MasterCommand, ETC_PDO_SIZE);
    ec_send_processdata();
}

//数组大小根据从站数量确定
MotorBlock pdo_rx[CAN_CHANNELS][MOTORS_PER_CH];
MotorBlock pdo_tx[CAN_CHANNELS][MOTORS_PER_CH];
SlaveChannel_fdBack fbBack_Message[SLAVE_NUMBER];
/**
 * @description:
 * @return {*}
 */
void EtherCAT_Run()
{
    if (ecat_phys < 0) {
        EtherCAT_Command_Set();
        return;
    }
    if (wkc_err_iteration_count > K_ETHERCAT_ERR_PERIOD)
    {
        wkc_err_count = 0;
        wkc_err_iteration_count = 0;
    }
    if (wkc_err_count > K_ETHERCAT_ERR_MAX)
    {
        //printf("[EtherCAT Error] Error count too high!\n");
        degraded_handler();
    }
    // send 
    EtherCAT_Command_Set();
    int rec = ec_send_processdata();
    // receiveSLAVE_NUMBER
    wkc = ec_receive_processdata(EC_TIMEOUTRET);
    EtherCAT_Data_Get();

    //check for dropped packet
    if (wkc < expectedWKC){
        //printf("\x1b[31m[EtherCAT Error] Dropped packet (Bad WKC!, WKC num = %i, expected = %i)\x1b[0m\n", wkc, expectedWKC);
        wkc_err_count++;
    }
    else{
        needlf = TRUE;
    }
    wkc_err_iteration_count++;
    // return;
}

/**
 * @description: slave data get
 * @return {*}
 * @author: Kx Zhang
 */
#define IFprintFEEDBACK (1)

void EtherCAT_Data_Get()
{
    memcpy(pdo_rx, ec_slave[ecat_phys].inputs, ETC_PDO_SIZE);

    // debug: print RX feedback decoded every 1000 cycles
    static int dbg_rx_cnt = 0;
    if (++dbg_rx_cnt >= 1000) {
        dbg_rx_cnt = 0;
        //printf("[PDO RX] ==================\n");
        for (int ch = 0; ch < ServoStruct.CAN_board_num; ch++) {
            int channel = ServoStruct.CAN_boards[ch].channel_idx;
            for (int j = 0; j < ServoStruct.CAN_boards[ch].JntNum_onCan; j++) {
                int slot = ServoStruct.CAN_boards[ch].motor_slot[j];
                int id = ServoStruct.CAN_boards[ch].can_jnt_ID[j];
                MotorBlock *mb = &pdo_rx[channel][slot];
                //printf("[PDO RX] CH%d SLOT%d ID=%-3d DLC=%d DATA=",
                //       channel, slot, id, mb->dlc);
                //for (int d = 0; d < mb->dlc; d++) printf("%02X ", mb->data[d]);
                //printf("\n");
                if (mb->dlc == 0) continue;
                uint8_t ack = mb->data[0] >> 5;
                uint8_t err = mb->data[0] & 0x1F;
                int pos_int = mb->data[1] << 8 | mb->data[2];
                int spd_int = mb->data[3] << 4 | ((mb->data[4] & 0xF0) >> 4);
                int cur_int = (mb->data[4] & 0x0F) << 8 | mb->data[5];
                float pos_rad = uint_to_float(pos_int, POS_MIN, POS_MAX, 16);
                float spd_rad = uint_to_float(spd_int, SPD_MIN, SPD_MAX, 12);
                float cur_a = uint_to_float(cur_int, I_MIN, I_MAX, 12);
                float temp_c = (mb->data[6] - 50.0f) / 2.0f;
                //printf("[PDO RX] CH%d SLOT%d ID=%-3d ack=%d err=0x%02X pos=%.4f spd=%.2f cur=%.2fA temp=%.0fC\n",
                //       channel, slot, id, ack, err, pos_rad, spd_rad, cur_a, temp_c);
            }
        }
    }

    for(int ch = 0; ch < ServoStruct.CAN_board_num; ch++){
        int channel = ServoStruct.CAN_boards[ch].channel_idx;
        for(int j = 0; j < ServoStruct.CAN_boards[ch].JntNum_onCan; j++){
            int slot = ServoStruct.CAN_boards[ch].motor_slot[j];
            uint8_t repack_rc = RV_can_data_repack(&pdo_rx[channel][slot], comm_ack, ch);
            if (repack_rc == 255) {
                // 无有效反馈帧（dlc==0 / 广播 / 未知帧）：跳过本轮读取。
                // 否则会读到 rv_motor_msg 里另一条通道同 CAN ID 电机的残留值，
                // 造成左右臂反馈互相污染（左右臂关节 CAN ID 均为 1~7）。
                continue;
            }
            int rv_ID = ServoStruct.CAN_boards[ch].can_jnt_ID[j] - 1;

            int motor_id = ServoStruct.CAN_boards[ch].can_jnt_ID[j];
            ServoStruct.CAN_boards[ch].can_jnt_ret_msg[j].joint_posRet = ServoStruct.CAN_boards[ch].can_jnt_direComp[j] * rv_motor_msg[rv_ID].angle_actual_rad;
            ServoStruct.CAN_boards[ch].can_jnt_ret_msg[j].joint_spdRet = ServoStruct.CAN_boards[ch].can_jnt_direComp[j] * rv_motor_msg[rv_ID].speed_actual_rad;
            ServoStruct.CAN_boards[ch].can_jnt_ret_msg[j].joint_currentRet = rv_motor_msg[rv_ID].current_actual_float;
            ServoStruct.CAN_boards[ch].can_jnt_ret_msg[j].joint_temperature = (rv_motor_msg[rv_ID].temperature - 50.0)/2.0;

            gripperFdbDecode(channel, motor_id, repack_rc,
                             rv_motor_msg[rv_ID], pdo_rx[channel][slot]);
        }
    }
}

/**
 * @description: slave command set
 * @return {*}
 * @author: Kx Zhang
 */
#define frequency 1000
#define POS_SPD 3.14/frequency
float pos_set = 0,delta_pos = POS_SPD;

extern std::vector<int> base_rigidityCtlMod;

//数据转换函数
uint16_t rads_to_rpm(double rad_per_sec) {
    const double RAD_TO_RPM = 60.0 / (2.0 * 3.14159265358979323846); // 60 / (2π)
    return static_cast<uint16_t>(std::abs(rad_per_sec * RAD_TO_RPM  * 10));
}

float rads_to_deg(double rad){
    float mRes = rad * 180 / M_PI;
    return mRes;
}


int gripperDireComp(int channel){
    for(const auto& board : ServoStruct.CAN_boards){
        if(board.channel_idx != channel){
            continue;
        }
        for(int joint = 0; joint < board.JntNum_onCan; ++joint){
            if(board.can_jnt_ID[joint] == 8){
                return board.can_jnt_direComp[joint];
            }
        }
    }
    return 0;
}

int CommandMODE = 1;         //命令模式，根据不同的命令模式执行不同的指令下发方式

void EtherCAT_Command_Set()
{
    // handle queue commands
    if (ecat_phys > 0 && messages[0].read_available()) {
        MotorBlock_ptr queue_msg;
        if (messages[0].pop(queue_msg) && queue_msg != nullptr) {
            memset(pdo_tx, 0, sizeof(pdo_tx));
            pdo_tx[0][0] = *queue_msg;
            memcpy(ec_slave[ecat_phys].outputs, pdo_tx, ETC_PDO_SIZE);
            printf("[Queue] Sent special command to ETC (phys_slave %d)\n", ecat_phys);
            return;
        }
    }

    // fill PDO for all channels
    memset(pdo_tx, 0, sizeof(pdo_tx));

    for(int ch = 0; ch < ServoStruct.CAN_board_num; ch++){
        int channel = ServoStruct.CAN_boards[ch].channel_idx;
        std::vector<int> motor_slot = ServoStruct.CAN_boards[ch].motor_slot;
        std::vector<int> jointId = ServoStruct.CAN_boards[ch].can_jnt_ID;
        std::vector<int> jnt_direComp = ServoStruct.CAN_boards[ch].can_jnt_direComp;
        std::vector<joint_desMsg> joint_des_msg = ServoStruct.CAN_boards[ch].can_jnt_des_msg;

        for(int j = 0; j < ServoStruct.CAN_boards[ch].JntNum_onCan; j++){
            int slot = motor_slot[j];
            int motor_id = jointId[j];
            double pos = jnt_direComp[j] * joint_des_msg[j].joint_pos;
            double spd = jnt_direComp[j] * joint_des_msg[j].joint_spd;
            double tor = jnt_direComp[j] * joint_des_msg[j].joint_tor;

            // gripper joints: pos/spd/tor/Kp/Kd from topic, channel区分左右
            if(motor_id == 8){
                gripperCmd *g = (channel == 0) ? &left_gripper_cmd : &right_gripper_cmd;
                int dir = jnt_direComp[j];

                if(g->joint_mode == 1){
                    send_motor_ctrl_cmd(&pdo_tx[channel][slot], motor_id,
                                        g->joint_Kp, g->joint_Kd,
                                        dir * g->joint_pos, 0.0, 0.0);
                    continue;
                }

                double raw_value = dir * g->joint_tor * 100.0;
                // 限幅到 int16_t 有效范围
                if (raw_value > 32767.0) {
                    raw_value = 32767.0;
                } else if (raw_value < -32768.0) {
                    raw_value = -32768.0;
                }
                int16_t torRes = (int16_t)round(raw_value); 

                set_motor_cur_tor(&pdo_tx[channel][slot], motor_id, torRes, 1, 1);
            }
            else if(joint_des_msg[j].transMod == 4){
                set_motor_position(&pdo_tx[channel][slot], motor_id, rads_to_deg(pos), rads_to_rpm(spd), 4000, 1);
            }else if(joint_des_msg[j].transMod == 5){
                set_motor_cur_tor(&pdo_tx[channel][slot], motor_id, tor * 100, 0, 1);
            }else if(motor_id != 8){
                send_motor_ctrl_cmd(&pdo_tx[channel][slot], motor_id, joint_des_msg[j].joint_Kp, joint_des_msg[j].joint_Kd, pos, spd, tor);
            }
        }
    }

    // debug: print TX commands decoded every 1000 cycles
    static int dbg_cnt = 0;
    if (++dbg_cnt >= 1000) {
        dbg_cnt = 0;
        /**
        printf("[PDO TX] ==================\n");
        for (int ch = 0; ch < ServoStruct.CAN_board_num; ch++) {
            int channel = ServoStruct.CAN_boards[ch].channel_idx;
            for (int j = 0; j < ServoStruct.CAN_boards[ch].JntNum_onCan; j++) {
                int slot = ServoStruct.CAN_boards[ch].motor_slot[j];
                int id = ServoStruct.CAN_boards[ch].can_jnt_ID[j];
                int dir = ServoStruct.CAN_boards[ch].can_jnt_direComp[j];
                MotorBlock *mb = &pdo_tx[channel][slot];
                printf("[PDO TX] CH%d SLOT%d ID=%-3d DLC=%d DATA=",
                       channel, slot, id, mb->dlc);
                for (int d = 0; d < mb->dlc; d++) printf("%02X ", mb->data[d]);
                printf("\n");
                // gripper reads from gripperCmd, arm reads from desMsg
                if(id == 8){
                    gripperCmd *g = (channel == 0) ? &left_gripper_cmd : &right_gripper_cmd;
                    printf("[PDO TX] CH%d SLOT%d ID=%-3d Kp=%.0f Kd=%.1f pos=%.4f spd=%.2f tor=%.2f\n",
                           channel, slot, id,
                           g->joint_Kp, g->joint_Kd,
                           dir * g->joint_pos,
                           dir * g->joint_spd,
                           dir * g->joint_tor);
                }else{
                    joint_desMsg *cmd = &ServoStruct.CAN_boards[ch].can_jnt_des_msg[j];
                    printf("[PDO TX] CH%d SLOT%d ID=%-3d Kp=%.0f Kd=%.1f pos=%.4f spd=%.2f tor=%.2f\n",
                           channel, slot, id,
                           cmd->joint_Kp, cmd->joint_Kd,
                           dir * cmd->joint_pos,
                           dir * cmd->joint_spd,
                           dir * cmd->joint_tor);
                }
            }
        }
        /**/
    }

    // send to ETC slave
    if (ecat_phys > 0) {
        memcpy(ec_slave[ecat_phys].outputs, pdo_tx, ETC_PDO_SIZE);
    }
}
/*  关节命令输入函数
*/
void EtherCAT_transInp(std::vector<std::vector<joint_desMsg>> allJnt_desMsg){
    for(int i = 0; i < CAN_and_Jnt_num.CAN_board_num; i++){
        for(int j = 0; j < CAN_and_Jnt_num.Jnt_num_on_each_board[i]; j++){
            ServoStruct.CAN_boards[i].can_jnt_des_msg[j] = allJnt_desMsg[i][j];
        }
    }
}
/*  关节反馈值输出函数
*/
std::vector<std::vector<joint_retMsg>> EtherCAT_transOutp(){
    std::vector<std::vector<joint_retMsg>> JntFeedBack;

    JntFeedBack.resize(CAN_and_Jnt_num.CAN_board_num);
    for(int i = 0; i < CAN_and_Jnt_num.CAN_board_num; i++){
        JntFeedBack[i].resize(CAN_and_Jnt_num.Jnt_num_on_each_board[i]);
        for(int j = 0; j < CAN_and_Jnt_num.Jnt_num_on_each_board[i]; j++){
            JntFeedBack[i][j] = ServoStruct.CAN_boards[i].can_jnt_ret_msg[j];
        }
    }

    return JntFeedBack;
}

//EtherCAT运行流程包装 -- 仅接收运行流程
std::vector<std::vector<joint_retMsg>> runMsgGet(){
    EtherCAT_direGetFdback();
    std::vector<std::vector<joint_retMsg>> msgGet = EtherCAT_transOutp();

    return msgGet;
}

//EtherCAT运行流程包装 -- 完整收发流程
std::vector<std::vector<joint_retMsg>> runImpl(std::vector<std::vector<joint_desMsg>> allJnt_desMsg) {
    EtherCAT_transInp(allJnt_desMsg);       //执行信息输入函数
    EtherCAT_Run();     //执行Ethercat通讯函数
    std::vector<std::vector<joint_retMsg>> msgRet = EtherCAT_transOutp();   //执行消息输出函数
    
    return msgRet;
}

// 假设 ec_slavecount 和 ec_slave[] 已定义（来自 EtherCAT 主站库）
extern int ec_slavecount;
extern ec_slavet ec_slave[];

// 函数声明
int ec_statecheck(uint16 slave, uint16 target_state, uint32 timeout);
int ec_writestate(uint16 slave);

void switch_slave_states() {
    // 1. 检查从站是否在 PRE_OP 状态
    std::cout << "===== 检查从站 PRE_OP 状态 =====" << std::endl;
    bool all_pre_op = true;
    std::cout<<"ec_slavecount = "<<ec_slavecount<<std::endl;
    for (int slave = 0; slave < ec_slavecount; ++slave) {
        if (ec_slave[slave + 1].state != EC_STATE_PRE_OP) {
            std::cerr << "[ERROR] 从站 " << slave << " 不在 PRE_OP 状态！当前状态: " 
                      << ec_slave[slave + 1].state << std::endl;
            all_pre_op = false;
        } else {
            std::cout << "从站 " << slave << " 处于 PRE_OP 状态" << std::endl;
        }
    }

    /*
    // 2. 切换到 SAFE_OP 状态
    std::cout << "\n===== 切换到 SAFE_OP 状态 =====" << std::endl;
    for (int slave = 0; slave < ec_slavecount; ++slave) {
        ec_slave[slave + 1].state = EC_STATE_SAFE_OP;
        ec_writestate(slave + 1); // 发送状态切换命令

        // 等待状态切换完成
        int ret1 = ec_statecheck(slave + 1, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
        if (ret1 != 4) { // ec_statecheck 返回 1 表示成功
            std::cerr << "[ERROR02] 从站 " << slave << " 切换 SAFE_OP 失败 (AL Status=0x" 
                      << std::hex << ec_slave[slave + 1].ALstatuscode << "), ret = " << ret1 << std::endl;
            // 尝试恢复：回退到 PRE_OP 或重新初始化
            ec_slave[slave + 1].state = EC_STATE_PRE_OP;
            ec_writestate(slave + 1);
        } else {
            std::cout << "从站 " << slave << " 切换到 SAFE_OP 成功" << std::endl;
        }
    }
    */

    // 3. 切换到 OPERATIONAL 状态
    std::cout << "\n===== 切换到 OPERATIONAL 状态 =====" << std::endl;
    for (int slave = 0; slave < ec_slavecount; ++slave) {
        ec_slave[slave + 1].state = EC_STATE_OPERATIONAL;
        ec_writestate(slave + 1); // 发送状态切换命令

        // 等待状态切换完成
        int ret2 = ec_statecheck(slave + 1, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
        if (ret2 != 8) {
            std::cerr << "[ERROR03] 从站 " << slave << " 切换 OPERATIONAL 失败 (AL Status=0x" 
                      << std::hex << ec_slave[slave + 1].ALstatuscode << "), ret = " << ret2 << std::endl;
            // 回退到 SAFE_OP
            ec_slave[slave + 1].state = EC_STATE_SAFE_OP;
            ec_writestate(slave + 1);
        } else {
            std::cout << "从站 " << slave << " 切换到 OPERATIONAL 成功" << std::endl;
        }
    }
}

void startRun() {
    std::cout << "开启transmit线程成功" << std::endl;
    running = true;
    //runThread = std::thread(runImpl);
    //usleep(10000);

    //读取yaml文件
    std::cout << "开始读取从站连接配置yaml文件" << std::endl;
    ServoStruct = load_servo_config(get_yaml_path());
    std::cout << "读取从站连接配置yaml文件完毕" << std::endl;

    CAN_and_Jnt_num.CAN_board_num = ServoStruct.CAN_board_num;
    CAN_and_Jnt_num.Jnt_num_on_each_board.assign(ServoStruct.CAN_board_num, 0);
    for(int i = 0; i < ServoStruct.CAN_board_num; i++){
        CAN_and_Jnt_num.Jnt_num_on_each_board[i] = ServoStruct.CAN_boards[i].JntNum_onCan;
    }

    //打印读取到下位机结构
    std::cout << "====================从站结构检查====================" << std::endl;
    std::cout << "打印读取到下位机基本结构" << std::endl;
    std::cout << "CAN板数量：" << ServoStruct.CAN_board_num << std::endl;
    for(int i = 0; i < ServoStruct.CAN_board_num; i++){
        std::cout << "对于CAN板：" << i+1 << std::endl;
        std::cout << "使用的CAN通道数目：" << ServoStruct.CAN_boards[i].can_num_activate << std::endl;
        std::cout << "使用的CAN id ： ";
        for(int j = 0; j < ServoStruct.CAN_boards[i].can_num_activate; j++){
            std::cout << ServoStruct.CAN_boards[i].can_id[j] << " " ;
        }
        std::cout << std::endl;
        std::cout << "每一个CAN上挂在关节数量：" ;
        for(int j = 0; j < ServoStruct.CAN_boards[i].can_num_activate; j++){
            std::cout << ServoStruct.CAN_boards[i].can_slave_num[j] << " " ;
        }
        std::cout << std::endl;

        std::cout << "板上关节数量：" << ServoStruct.CAN_boards[i].JntNum_onCan << std::endl;
        std::cout << "板上关节ID依次为：" ;
        for(int j = 0; j < ServoStruct.CAN_boards[i].JntNum_onCan; j++){
            std::cout << ServoStruct.CAN_boards[i].can_jnt_ID[j] << " " ;
        }
        std::cout << std::endl;
        std::cout << "==================================================" << std::endl;
    }
    
    std::cout << "跳转执行模式刷新" << std::endl;
    switch_slave_states();

    // 动态检测 ECAT-8CAN 从站
    for (int slave = 0; slave < ec_slavecount; slave++) {
        int phys = slave + 1;
        if (ec_slave[phys].eep_man == ETC_VENDOR_ID &&
            ec_slave[phys].eep_id == ETC_PRODUCT_CODE) {
            ecat_phys = phys;
            printf("[ETC] Found at slave %d (Vendor=0x%08X Product=0x%08X)\n",
                   phys, ec_slave[phys].eep_man, ec_slave[phys].eep_id);
            break;
        }
    }
    if (ecat_phys < 0) {
        printf("[ETC] WARNING: ECAT-8CAN gateway not found!\n");
    } else {
        printf("[ETC] PDO size: IN=%d bytes (%d slots)  OUT=%d bytes (%d slots)\n",
               ec_slave[ecat_phys].Ibytes,
               ec_slave[ecat_phys].Ibytes / 14,
               ec_slave[ecat_phys].Obytes,
               ec_slave[ecat_phys].Obytes / 14);
        printf("[ETC] Expected: %d bytes (%d slots)\n", ETC_PDO_SIZE, ETC_PDO_SIZE / 14);
    }

    // ===== 1. 检查所有从站状态 =====
    printf("[myEtherCAT] Checking slave states again before communication...\n");
    for (int slave = 0; slave < ec_slavecount; ++slave) {
        printf("Slave %d: State=0x%02X, AL Status=0x%04X\n",
            slave, ec_slave[slave + 1].state, ec_slave[slave + 1].ALstatuscode);
    }
    //std::cout << "模式刷新完毕，打印初始关节位置：" << std::endl;
}

//辅助函数 -- 返回CAN板数量以及各个CAN板上关节数量的结构体
CAN_JNT_numStruct CAN_JNT_arrange_outp(){
    return CAN_and_Jnt_num;
}

//辅助函数 -- 获取关节Kt
std::vector<double> joint_Kt_get(){
    std::vector<double> mRes;

    for(int i = 0; i < CAN_and_Jnt_num.CAN_board_num; i++){
        for(int j = 0; j < CAN_and_Jnt_num.Jnt_num_on_each_board[i]; j++){
            mRes.push_back(ServoStruct.CAN_boards[i].can_jnt_Kt[j]);
        }
    }

    return mRes;
}

//辅助函数：数据类型转换
float fdbackTrans(uint16_t inpData, float lower_bound, float upper_bound, uint16_t range)
{
    if(inpData > range){
        std::cerr << "error03: dataTrans beyond range." << std::endl;
        return 0;
    }
    return lower_bound + inpData*(upper_bound - lower_bound)/range;
}
float IqRange[2] = {-30, 30};

//---------------调试辅助函数---------------

//输出容器中关节目标信息
//版本1 -- 直接按照原始结构输出
std::vector<std::vector<joint_desMsg>> JntDesMsg_inContainer_Outp_v1(){
    std::vector<std::vector<joint_desMsg>> allJnt_DesMsg;

    allJnt_DesMsg.resize(ServoStruct.CAN_boards.size());
    for(int i = 0; i < ServoStruct.CAN_boards.size(); i++){
        allJnt_DesMsg[i] = ServoStruct.CAN_boards[i].can_jnt_des_msg;
    }

    return allJnt_DesMsg;
}


// 夹爪控制接口实现 (力控模式)
void setLeftGripperCmd(const gripperCmd& cmd){
    left_gripper_cmd = cmd;
}

void setRightGripperCmd(const gripperCmd& cmd){
    right_gripper_cmd = cmd;
}

//----------------------------------------
