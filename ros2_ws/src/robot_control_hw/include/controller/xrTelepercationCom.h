#ifndef __xrTelepercationCom__H__
#define __xrTelepercationCom__H__

#include <cstdio>
#include <csignal>
#include <cstring>
#include <atomic>
#include <string>
#include <vector>
#include <iostream>
#include <thread>

#include <deque>
#include <algorithm>
#include <numeric>
#include <cmath>

#include <unistd.h> // for read()
#include <sys/select.h> // for select()

#include "robot_control/msgType.h"
#include "controller/upper_Transmit.h"

/*
        by 霜云喵
==========================
遥操作通讯代码
==========================

==========================
*/
extern std::vector<double> dbg_reserve;

//------------------------------------------------------------
//辅助类：插补器
/*插补器*/
class stTraj_runiner {
    /*插补器*/
    /*注意总运动时间必须是control_T的整数倍*/
    private:
        float control_T = TempTime;//控制周期

        float lastTempPosOutp = 0.0f;
        float lastTempVelOutp = 0.0f;
    public:
        /**拟合代码初始化代码
         * deg -- 样条阶数
         * bar_len -- 窗口长度
        */

        void Init(){
            control_T = TempTime;
            lastTempPosOutp = 0.0f;
            lastTempVelOutp = 0.0f;
        };

        //点位插补
        TrajResult dPos_queue_stabTime_dot(float mposDes, float mTrajTime, float accTimeRateBase = 1.2);
        /*备注：最后一个参量的含义为加速时间占到达总位移一半时的总运动时间的比例，这个数值必须大于1*/

        //辅助函数 -- 输出最后一轮的速度
        //上述计算过程中的mposDes指的是相对于初始位置的位置变化量，因此这里引入初始位置将位置量还原
        std::pair<float, float> lastPosVelGet(float posSt = 0.0f);
};

//------------------------------------------------------------
//通讯核心代码
class xr_telepercation_ctl{
    private:
        //过程变量（最终的输出六维数组）
        std::vector<double> lArm_end_pos_rpy;
        std::vector<double> rArm_end_pos_rpy;

        //控制变量
        dataSendWrap mdataSend;
        ctlDesMsgWrap ctlStMsg;                     //初始位置包装
        ctlDesMsgWrap ctlDesMsg;                    //目标位置包装

        //初始位置偏置
        std::vector<double> st_pos_l;
        std::vector<double> st_pos_r;
        //初始姿态偏置
        std::vector<double> st_gesture_l;
        std::vector<double> st_gesture_r;

        // 累积的相对姿态。每帧增量先转四元数再复合，避免欧拉角逐项
        // 相加带来的轴耦合和万向节锁问题。
        std::vector<double> l_arm_quat_accum;
        std::vector<double> r_arm_quat_accum;
        // Last emitted ZYX angles, used only to unwrap +/-pi crossings before
        // sending the Cartesian RPY message to the lower controller.
        std::vector<double> last_l_arm_zyx;
        std::vector<double> last_r_arm_zyx;
        bool last_arm_zyx_valid = false;

        // VR 最终末端目标限位：位置单位 m，姿态为 RPY [rad]。
        // position_enabled/orientation_enabled 可独立关闭各自限幅。
        // 从 ctl_msgs_hw/config/controllerConfig.yaml 的 vrTeleoperationLimit 读取。
        bool position_limit_enabled = true;
        bool orientation_limit_enabled = true;
        std::vector<double> l_pos_min;
        std::vector<double> l_pos_max;
        std::vector<double> r_pos_min;
        std::vector<double> r_pos_max;
        std::vector<double> l_rpy_min;
        std::vector<double> l_rpy_max;
        std::vector<double> r_rpy_min;
        std::vector<double> r_rpy_max;
        bool l_position_limit_active = false;
        bool r_position_limit_active = false;
        bool l_orientation_limit_active = false;
        bool r_orientation_limit_active = false;

        void load_teleoperation_limits();
        void limit_and_reanchor_target(
            std::vector<double>& target_rpy,
            std::vector<double>& position_delta,
            std::vector<double>& orientation_accumulator,
            const std::vector<double>& base_position,
            const std::vector<double>& base_rpy,
            const std::vector<double>& position_min,
            const std::vector<double>& position_max,
            const std::vector<double>& rpy_min,
            const std::vector<double>& rpy_max,
            bool position_limit_enabled,
            bool orientation_limit_enabled,
            bool& position_limit_active,
            bool& orientation_limit_active,
            const char* arm_name);

        //刚度参数：
        int rigidity_mod = 2;   //(定义为中刚度模式)

        void trans_and_outp_core();

        //过程控制变量：

        //是否为初始化轨迹阶段
        bool ifUseStTrajOutp = true;
        int zero_outputTimes = 0;
        int total_zeroTimes = 100;

        //是否为第一次输出
        bool ifFirstOutp = true;

        //---------- A 键关节空间回零状态 ----------
        //A 键触发标志（由 /vr/zero_return_cmd 回调置位，主循环消费）
        bool returnRequested = false;
        //是否正在执行关节空间回零轨迹
        bool returning = false;
        // Synchronize once from feedback when Cartesian teleoperation resumes
        // after A-key joint return, then go back to planner-state tracking.
        bool use_feedback_once_after_return = false;
        //末端笛卡尔位姿反馈缓存（回零完成后重新锚定笛卡尔遥操作基准）
        std::vector<double> l_desc_ret_cache;
        std::vector<double> r_desc_ret_cache;
        bool desc_ret_valid = false;

        //构建关节空间回零指令：mode 1（关节模式）+ 点位插补。
        //起点由中层节点依据当前反馈确定（ifUseRet=1），目标为 zeropos.yaml
        void build_joint_return_cmd();

        // A 键回零参数，从 controllerConfig.yaml 的 vrJointReturn 读取。
        double joint_return_trajectory_time_sec = 6.0;
        double joint_return_acceleration_time_ratio = 1.10;
        int joint_return_start_stable_cycles = 15;
        int joint_return_end_stable_cycles = 25;
        int joint_return_remaining_cycles = 0;

    public:

        //遥操作回调变量

        //左右臂姿态（欧拉四元数）
        std::vector<double> l_arm_end_pos;
        std::vector<double> r_arm_end_pos;

        //左右臂末端位置
        std::vector<double> l_arm_end_delta;
        std::vector<double> r_arm_end_delta;

        //A 键触发关节空间回零（由回零触发回调调用，仅置位请求标志）
        void trigger_joint_return();

        // 接收一帧 [yaw, pitch, roll] 姿态增量（弧度），按局部坐标复合。
        void apply_left_orientation_delta(const std::vector<double>& delta_zyx);
        void apply_right_orientation_delta(const std::vector<double>& delta_zyx);

        //缓存末端笛卡尔位姿反馈（由末端反馈回调调用）
        void update_desc_ret(const std::vector<double>& l_desc_pose, const std::vector<double>& r_desc_pose);

        //变量初始化
        void init();

        //变量转化并输出
        dataSendWrap trans_and_outp();

        //输出过程控制参量：初始静止状态；
        bool ifUseStFtr = true;
        int ststabTimes = 20;
        int stabtimecount = 0;
};

#endif
