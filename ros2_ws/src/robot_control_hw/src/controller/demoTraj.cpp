#include "controller/demoTraj.h"


//===============================================================================================

//--------------------插补类：--------------------

//一次插补
TrajResult runiner::dPos_queue_stabTime_single(float mposDes, float mTrajTime)
{
    //圆整时间
    int tempTimes = std::round(mTrajTime/this->control_T);
    float trajTime_round = this->control_T;

    //计算位置增量与速度
    float derPos = mposDes / tempTimes;
    float vel = mposDes / trajTime_round;

    //装入结构体
    TrajResult mRes;
    float tempPos = 0;
    for(int i = 0; i < tempTimes; i++){
    //for(int i = 0; i < 1; i++){
        
        tempPos = tempPos + derPos;
        mRes.positionIncrements.push_back(tempPos);
        mRes.VelocityTraj.push_back(vel);
    }

    //记录最后一轮的速度
    this->lastTempPosOutp = *mRes.positionIncrements.end();
    this->lastTempVelOutp = *mRes.VelocityTraj.end();

    return mRes;
}

//二次插补
TrajResult runiner::dPos_queue_stabTime_double(float mposDes, float velSt, float mtrajTime)
{
    //注意：输入量均为弧度制
    float posDes_half;
    float trajTime;
    //计算半切轨迹
    posDes_half = mposDes;
    trajTime = mtrajTime;

    //计算半切轨迹
    //核心——保证始末加速度为0（或近似0），末位置恰好达到目标位置
    float a = 12 * (velSt*trajTime - posDes_half) / std::pow(trajTime, 4);
    float b = a * trajTime;

    //据此生成增量轨迹
    std::vector<float> mResTraj;
    std::vector<float> mVelTraj;
    int m_tempLen = std::floor(trajTime / control_T);
    m_tempLen = std::max(1, m_tempLen); 

    float temp_T = this->control_T;
    for(int i = 0; i < m_tempLen; i++){
        float temp_pos = std::pow(temp_T, 4)*a*1.0/12 - std::pow(temp_T, 3)*b*1.0/6 + velSt*temp_T;
        float temp_vel = a * std::pow(temp_T, 3) - b * std::pow(temp_T, 2) / 2 + velSt;
        mResTraj.push_back(temp_pos);
        //mVelTraj.push_back(temp_vel);
        mVelTraj.push_back(0.0f);
        temp_T += control_T;
    }

    TrajResult mRes;
    mRes.positionIncrements = mResTraj;
    mRes.VelocityTraj = mVelTraj;

    this->lastTempPosOutp = *mRes.positionIncrements.end();
    this->lastTempVelOutp = *mRes.VelocityTraj.end();

    //return mRes_Interpolated;
    return mRes;
}

//点位插补
TrajResult runiner::dPos_queue_stabTime_dot(float mposDes, float mTrajTime, float accTimeRateBase)
{
    //轨迹一半的时间用来加速
    //由曲线推到可得，当加速过程时间占到时间的一半时，按照钟型曲线加速过程和匀速过程走过距离比例为1：2

    float t_half = mTrajTime / 2;
    float posDes_half = mposDes / 2;

    //计算半切轨迹及其增量
    std::vector<float> half_traj;
    std::vector<float> derta_half_traj;
    std::vector<float> half_vel;

    //轨迹参数计算a, b, 和v_max
    if (accTimeRateBase <= 0.5) {
        std::cerr << "accTimeRateBase must be greater than 0.5 to avoid division by zero" << std::endl;
        accTimeRateBase = 0.6; // 设置默认安全值
    }
    float t_acc = t_half / accTimeRateBase;
    float t_stab_half = t_half - t_acc;
    float denominator = std::pow(t_half, 4) * (2*accTimeRateBase - 1);
    float a = -12 * std::pow(accTimeRateBase, 4) * posDes_half / denominator;
    float b = a * t_half / accTimeRateBase;
    //float v_max = (1.0/3.0)*a*std::pow(t_acc, 3) - (1.0/2.0)*b*std::pow(t_acc,2);

    float acc_moveLen = (1.0/12)*a*std::pow(t_acc, 4) - (1.0/6)*b*std::pow(t_acc, 3);
    float v_max = (posDes_half - acc_moveLen) / t_stab_half;

    //半切轨迹及其增量计算
    int m_tempLen = std::round(t_half/this->control_T);
    float temp_T = this->control_T;
    float temp_pos_lst = 0;
    for(int i = 0; i < m_tempLen; i++){
        float temp_pos;
        if(temp_T < t_acc || temp_T == t_acc){
            temp_pos = (1.0/12)*a*std::pow(temp_T, 4) - (1.0/6)*b*std::pow(temp_T, 3);
        }else{
            temp_pos = temp_pos_lst + v_max*control_T;
        }
        half_traj.push_back(temp_pos);
        derta_half_traj.push_back(temp_pos - temp_pos_lst);
        half_vel.push_back((temp_pos - temp_pos_lst)/control_T);
        temp_pos_lst = temp_pos;
        temp_T = temp_T + this->control_T;
    }

    //依据半切轨迹计算全切轨迹
    float m_temp_pos = temp_pos_lst;
    float m_temp_pos_lst = temp_pos_lst;
    std::vector<float> all_traj = half_traj;
    std::vector<float> all_vel = half_vel;
    for(int i = m_tempLen - 1 ; i >= 0; i--){
        m_temp_pos += derta_half_traj.at(i);
        all_traj.push_back(m_temp_pos);
        all_vel.push_back((m_temp_pos-m_temp_pos_lst)/control_T);
        m_temp_pos_lst = m_temp_pos;
    }
    //all_traj.push_back(mposDes);
    //all_vel.push_back((mposDes - m_temp_pos)/control_T);

    TrajResult mRes;
    mRes.positionIncrements = all_traj;
    mRes.VelocityTraj = all_vel;

    //获取最后一轮的位置和速度
    lastTempPosOutp = mposDes;
    lastTempVelOutp = 0.0f;

    return mRes;
}

std::pair<float, float> runiner::lastPosVelGet(float posSt)
{
    float posOutpCor = this->lastTempPosOutp + posSt;
    std::pair<float, float> mRes = {posOutpCor, this->lastTempVelOutp};
    return mRes;
}

//-----------------------------------------------------------------------------------------
//辅助函数

std::vector<double> QuaternionPos_trans(std::vector<double> descPos){
    std::vector<double> mRes;
    Pose Qua = rpyToQuaternion(
        descPos[0], descPos[1], descPos[2], descPos[3], descPos[4], descPos[5]
    );
    mRes = {
        Qua.x, Qua.y, Qua.z,
        Qua.q.w, Qua.q.x, Qua.q.y, Qua.q.z
    };

    return mRes;
}

//-----------------------------------------------------------------------------------------
//核心类定义

//--------------------------------------------
//轨迹1：

void allDemoTraj::init_v1()
{
    //连续轨迹计算函数

    //关节起始位置
    //关节空间
    std::vector<double> lArm_stJntPos, rArm_stJntPos;
    lArm_stJntPos = lArm_zeroPos;
    rArm_stJntPos = rArm_zeroPos;
    rArm_stJntPos = rArm_zeroPos;
    //笛卡尔空间
    std::vector<double> lArm_stDescPos, rArm_stDescPos;
    lArm_stDescPos = l_descZeroPos;
    rArm_stDescPos = r_descZeroPos;

    ctlStMsg_v1.desPos_l_Descartes = lArm_stDescPos;
    ctlStMsg_v1.desPos_r_Descartes = rArm_stDescPos;
    ctlStMsg_v1.desPos_l_JointSpace = lArm_stJntPos;
    ctlStMsg_v1.desPos_r_JointSpace = rArm_stJntPos;

    ctlStMsg_v1.desTor_l_Descartes.assign(6, 0.0);
    ctlStMsg_v1.desTor_r_Descartes.assign(6, 0.0);
    ctlStMsg_v1.desTor_l_JointSpace.assign(LARMDOF, 0.0);
    ctlStMsg_v1.desTor_r_JointSpace.assign(RARMDOF, 0.0);

    //关节目标位置
    //关节空间的目标位置
    std::vector<double> lArm_desJntPos, rArm_desJntPos;
    lArm_desJntPos = {
        0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5
    };
    rArm_desJntPos = {
        //0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5
        0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5
    };
    //笛卡尔空间目标位置
    std::vector<double> lArm_desDescPos, rArm_desDescPos;
    lArm_desDescPos = {
        0.3, 0.144998, -0.16, -1.7, -0.8, 0.0 
    };
    rArm_desDescPos = {
        0.3, -0.144998, -0.16, 1.7, -0.8, 0.0
    };

    ctlDesMsg_v1.desPos_l_Descartes = lArm_desDescPos;
    ctlDesMsg_v1.desPos_r_Descartes = rArm_desDescPos;
    ctlDesMsg_v1.desPos_l_JointSpace = lArm_desJntPos;
    ctlDesMsg_v1.desPos_r_JointSpace = rArm_desJntPos;

    ctlDesMsg_v1.desTor_l_Descartes.assign(6, 0.0);
    ctlDesMsg_v1.desTor_r_Descartes.assign(6, 0.0);
    ctlDesMsg_v1.desTor_l_JointSpace.assign(LARMDOF, 0.0);
    ctlDesMsg_v1.desTor_r_JointSpace.assign(RARMDOF, 0.0);

    //角度输出模式是否为静止的
    std::vector<int> lArm_ifJntStab, rArm_ifJntStab;
    lArm_ifJntStab = { 
        //0, 0, 0, 0, 0, 0, 0 
        1, 1, 1, 1, 1, 1, 1 
    };
    rArm_ifJntStab = { 
        //0, 0, 0, 0, 0, 0, 0 
        1, 1, 1, 1, 1, 1, 0 
    };

    //控制值输出初始化

    //关节限位控制模式
    mdataSend.armTrajMsg.JntStabMod_l.ifUseJntLim = 0;
    mdataSend.armTrajMsg.JntStabMod_l.limJntNum = 1;
    mdataSend.armTrajMsg.JntStabMod_l.limJntID = {3};
    mdataSend.armTrajMsg.JntStabMod_l.limJntAng = {0.1};

    mdataSend.armTrajMsg.JntStabMod_r.ifUseJntLim = 0;
    mdataSend.armTrajMsg.JntStabMod_r.limJntNum = 1;
    mdataSend.armTrajMsg.JntStabMod_r.limJntID = {3};
    mdataSend.armTrajMsg.JntStabMod_r.limJntAng = {-0.1};

    //运动控制模式
    mdataSend.armTrajMsg.ctlTransMod = 1;
    mdataSend.armTrajMsg.TorFeedforwardCtl = 0;

    //控制量初始化
    mdataSend.armTrajMsg.ctlDesMsg.lArm_rigidity_mod.assign(7, 1);
    mdataSend.armTrajMsg.ctlDesMsg.rArm_rigidity_mod.assign(7, 1);
    mdataSend.armTrajMsg.ctlDesMsg.lArm_JntKp = lArm_defaultKp_low;
    mdataSend.armTrajMsg.ctlDesMsg.lArm_JntKd = lArm_defaultKd_low;
    mdataSend.armTrajMsg.ctlDesMsg.rArm_JntKp = rArm_defaultKp_low;
    mdataSend.armTrajMsg.ctlDesMsg.rArm_JntKd = rArm_defaultKd_low;

    //单点控制初始化
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.ifUseTemp = 1;         //是否使用循环输出

    double dotTrajTime = 2.0;                                               //轨迹时间2.0s
    double accRate = 1.2;                                                   //加速时间占比
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.JntTrajTime_L.assign(LARMDOF, dotTrajTime);
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.JntTrajTime_R.assign(RARMDOF, dotTrajTime);
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.JntAccRate_L.assign(LARMDOF, accRate);
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.JntAccRate_R.assign(RARMDOF, accRate);

    //单点位静止消息控制
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.ifReDefineStabJnt = false;
    runinStabMsgWrap runin_stab_msg;
    runin_stab_msg.ifUseStab = true;
    runin_stab_msg.start_stabTimes = 50;
    runin_stab_msg.des_stabTimes = 50;
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.lArm_stabMsg.assign(LARMDOF, runin_stab_msg);
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.rArm_stabMsg.assign(RARMDOF, runin_stab_msg);

    //控制量赋值
    mdataSend.armTrajMsg.ctlDesMsg.ifInitUseRet = 0;        //是否使用反馈值

    if(mdataSend.armTrajMsg.ctlTransMod == 1 || mdataSend.armTrajMsg.ctlTransMod == 3){
        mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg = lArm_stJntPos;
        mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg = rArm_stJntPos;
    }else{
        mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg = lArm_stDescPos;
        mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg = rArm_stDescPos;
    }
    mdataSend.armTrajMsg.ctlDesMsg.lArm_desTorMsg.assign(LARMDOF, 0.0);
    mdataSend.armTrajMsg.ctlDesMsg.rArm_desTorMsg.assign(RARMDOF, 0.0);
    mdataSend.armTrajMsg.ctlDesMsg.lArm_MDP_desJntVel.assign(LARMDOF, 0.0);
    mdataSend.armTrajMsg.ctlDesMsg.rArm_MDP_desJntVel.assign(RARMDOF, 0.0);

    for(int i = 0; i < LARMDOF; i++){
        mdataSend.armTrajMsg.ctlDesMsg.lArm_rigidity_mod[i] = m_rigidityCtlMod;
        mdataSend.armTrajMsg.ctlDesMsg.rArm_rigidity_mod[i] = m_rigidityCtlMod;
    }

    //针对底盘控制进行赋值
    mdataSend.baseCtlMsg.chassis_cmd.linear_x = 0.0;
    mdataSend.baseCtlMsg.chassis_cmd.linear_y = 0.0;
    mdataSend.baseCtlMsg.chassis_cmd.angular_z = 0.0;

    mdataSend.baseCtlMsg.lifter_cmd.control_mod = 0;        //默认处于遥操作模式
    mdataSend.baseCtlMsg.lifter_cmd.move_cmd = 0;
    mdataSend.baseCtlMsg.lifter_cmd.target_position = 1;
    mdataSend.baseCtlMsg.lifter_cmd.save_preset = false;    

    mdataSend.l_ifJntStab = lArm_ifJntStab;
    mdataSend.r_ifJntStab = rArm_ifJntStab;
}

DotOutpWrap allDemoTraj::TrajOutp_v1()
{
    //控制信息输出
    if(mdataSend.armTrajMsg.ctlTransMod == 1){
        mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg = ctlDesMsg_v1.desPos_l_JointSpace;
        mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg = ctlDesMsg_v1.desPos_r_JointSpace;
    }else{
        mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg = ctlDesMsg_v1.desPos_l_Descartes;
        mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg = ctlDesMsg_v1.desPos_r_Descartes;
    }
    return mdataSend;
}

//----------轨迹2----------
//定位精度测试轨迹

void allDemoTraj::init_v2()
{
    //连续轨迹计算函数

    //关节起始位置
    //关节空间
    std::vector<double> lArm_stJntPos, rArm_stJntPos;
    lArm_stJntPos = lArm_zeroPos;
    rArm_stJntPos = rArm_zeroPos;
    rArm_stJntPos = rArm_zeroPos;
    //笛卡尔空间
    std::vector<double> lArm_stDescPos, rArm_stDescPos;
    lArm_stDescPos = l_descZeroPos;
    rArm_stDescPos = r_descZeroPos;

    ctlStMsg_v2.desPos_l_Descartes = lArm_stDescPos;
    ctlStMsg_v2.desPos_r_Descartes = rArm_stDescPos;
    ctlStMsg_v2.desPos_l_JointSpace = lArm_stJntPos;
    ctlStMsg_v2.desPos_r_JointSpace = rArm_stJntPos;

    ctlStMsg_v2.desTor_l_Descartes.assign(6, 0.0);
    ctlStMsg_v2.desTor_r_Descartes.assign(6, 0.0);
    ctlStMsg_v2.desTor_l_JointSpace.assign(LARMDOF, 0.0);
    ctlStMsg_v2.desTor_r_JointSpace.assign(RARMDOF, 0.0);

    //关节目标位置1
    //关节空间的目标位置
    std::vector<double> lArm_desJntPos_1, rArm_desJntPos_1;
    lArm_desJntPos_1 = {
        0.0, 0.0, 0.0, -1.57, 0.0, 0.0, 0.0
    };
    rArm_desJntPos_1 = {
        0.0, 0.0, 0.0, -1.57, 0.0, 0.0, 0.0
    };
    //笛卡尔空间目标位置
    std::vector<double> lArm_desDescPos_1, rArm_desDescPos_1;
    lArm_desDescPos_1 = {
        0.23, 0.255, -0.309, 0.0, -1.5707, 0.0
    };
    rArm_desDescPos_1 = {
        0.23, -0.255, -0.309, 0.0, -1.5707, 0.0
    };

    ctlDesMsg_v2_p1.desPos_l_Descartes = lArm_desDescPos_1;
    ctlDesMsg_v2_p1.desPos_r_Descartes = rArm_desDescPos_1;
    ctlDesMsg_v2_p1.desPos_l_JointSpace = lArm_desJntPos_1;
    ctlDesMsg_v2_p1.desPos_r_JointSpace = rArm_desJntPos_1;

    ctlDesMsg_v2_p1.desTor_l_Descartes.assign(6, 0.0);
    ctlDesMsg_v2_p1.desTor_r_Descartes.assign(6, 0.0);
    ctlDesMsg_v2_p1.desTor_l_JointSpace.assign(LARMDOF, 0.0);
    ctlDesMsg_v2_p1.desTor_r_JointSpace.assign(RARMDOF, 0.0);

    //角度输出模式是否为静止的
    lArm_ifJntStab_v2_1 = { 1, 1, 1, 0, 1, 1, 1 };
    rArm_ifJntStab_v2_1 = { 1, 1, 1, 0, 1, 1, 1 };

    //关节目标位置2
    //关节空间的目标位置
    std::vector<double> lArm_desJntPos_2, rArm_desJntPos_2;
    lArm_desJntPos_2 = {
        -0.5, 0.0, 0.0, 1.07, 0.0, 0.0, 0.0
    };
    rArm_desJntPos_2 = {
        -0.5, 0.0, 0.0, -1.07, 0.0, 0.0, 0.0
    };
    //笛卡尔空间目标位置
    std::vector<double> lArm_desDescPos_2, rArm_desDescPos_2;
    lArm_desDescPos_2 = {
        0.35, 0.255, -0.309, 0.0, -1.5707, 0.0
    };
    rArm_desDescPos_2 = {
        0.35, -0.255, -0.309, 0.0, -1.5707, 0.0
    };

    ctlDesMsg_v2_p2.desPos_l_Descartes = lArm_desDescPos_2;
    ctlDesMsg_v2_p2.desPos_r_Descartes = rArm_desDescPos_2;
    ctlDesMsg_v2_p2.desPos_l_JointSpace = lArm_desJntPos_2;
    ctlDesMsg_v2_p2.desPos_r_JointSpace = rArm_desJntPos_2;

    ctlDesMsg_v2_p2.desTor_l_Descartes.assign(6, 0.0);
    ctlDesMsg_v2_p2.desTor_r_Descartes.assign(6, 0.0);
    ctlDesMsg_v2_p2.desTor_l_JointSpace.assign(LARMDOF, 0.0);
    ctlDesMsg_v2_p2.desTor_r_JointSpace.assign(RARMDOF, 0.0);

    //角度输出模式是否为静止的
    lArm_ifJntStab_v2_2 = { 0, 1, 1, 0, 1, 1, 1 };
    rArm_ifJntStab_v2_2 = { 0, 1, 1, 0, 1, 1, 1 };

    //控制值输出初始化

    //关节限位控制模式
    mdataSend.armTrajMsg.JntStabMod_l.ifUseJntLim = 0;
    mdataSend.armTrajMsg.JntStabMod_l.limJntNum = 1;
    mdataSend.armTrajMsg.JntStabMod_l.limJntID = {3};
    mdataSend.armTrajMsg.JntStabMod_l.limJntAng = {0.1};

    mdataSend.armTrajMsg.JntStabMod_r.ifUseJntLim = 0;
    mdataSend.armTrajMsg.JntStabMod_r.limJntNum = 1;
    mdataSend.armTrajMsg.JntStabMod_r.limJntID = {3};
    mdataSend.armTrajMsg.JntStabMod_r.limJntAng = {-0.1};

    //运动控制模式
    mdataSend.armTrajMsg.ctlTransMod = 1;
    mdataSend.armTrajMsg.TorFeedforwardCtl = 0;

    //控制量初始化
    mdataSend.armTrajMsg.ctlDesMsg.lArm_rigidity_mod.assign(7, 1);
    mdataSend.armTrajMsg.ctlDesMsg.rArm_rigidity_mod.assign(7, 1);
    mdataSend.armTrajMsg.ctlDesMsg.lArm_JntKp = lArm_defaultKp_low;
    mdataSend.armTrajMsg.ctlDesMsg.lArm_JntKd = lArm_defaultKd_low;
    mdataSend.armTrajMsg.ctlDesMsg.rArm_JntKp = rArm_defaultKp_low;
    mdataSend.armTrajMsg.ctlDesMsg.rArm_JntKd = rArm_defaultKd_low;

    //单点控制初始化
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.ifUseTemp = 0;         //是否使用循环输出

    double dotTrajTime = 1.0;                                               //轨迹时间2.0s
    double accRate = 1.2;                                                   //加速时间占比
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.JntTrajTime_L.assign(LARMDOF, dotTrajTime);
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.JntTrajTime_R.assign(RARMDOF, dotTrajTime);
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.JntAccRate_L.assign(LARMDOF, accRate);
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.JntAccRate_R.assign(RARMDOF, accRate);

    //单点位静止消息控制
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.ifReDefineStabJnt = false;
    runinStabMsgWrap runin_stab_msg;
    runin_stab_msg.ifUseStab = true;
    runin_stab_msg.start_stabTimes = 50;
    runin_stab_msg.des_stabTimes = 50;
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.lArm_stabMsg.assign(LARMDOF, runin_stab_msg);
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.rArm_stabMsg.assign(RARMDOF, runin_stab_msg);

    //控制量赋值
    mdataSend.armTrajMsg.ctlDesMsg.ifInitUseRet = 0;        //是否使用反馈值

    if(mdataSend.armTrajMsg.ctlTransMod == 1 || mdataSend.armTrajMsg.ctlTransMod == 3){

        mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg = lArm_stJntPos;
        mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg = rArm_stJntPos;

    }else{

        Pose lArm_Quaternion_pos, rArm_Quaternion_pos;
        lArm_Quaternion_pos = rpyToQuaternion(
            lArm_stDescPos[0], lArm_stDescPos[1], lArm_stDescPos[2], 
            lArm_stDescPos[3], lArm_stDescPos[4], lArm_stDescPos[5]
        );
        rArm_Quaternion_pos = rpyToQuaternion(
            rArm_stDescPos[0], rArm_stDescPos[1], rArm_stDescPos[2],
            rArm_stDescPos[3], rArm_stDescPos[4], rArm_stDescPos[5]
        );

        std::vector<double> lArm_desDescQuaPos, rArm_desDescQuaPos;
        lArm_desDescQuaPos = {
            lArm_Quaternion_pos.x, lArm_Quaternion_pos.y, lArm_Quaternion_pos.z,
            lArm_Quaternion_pos.q.w, lArm_Quaternion_pos.q.x, lArm_Quaternion_pos.q.y, lArm_Quaternion_pos.q.z
        };
        rArm_desDescQuaPos = {
            rArm_Quaternion_pos.x, rArm_Quaternion_pos.y, rArm_Quaternion_pos.z,
            rArm_Quaternion_pos.q.w, rArm_Quaternion_pos.q.x, rArm_Quaternion_pos.q.y, rArm_Quaternion_pos.q.z
        };

        mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg = lArm_desDescQuaPos;
        mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg = rArm_desDescQuaPos;

    }
    mdataSend.armTrajMsg.ctlDesMsg.lArm_desTorMsg.assign(LARMDOF, 0.0);
    mdataSend.armTrajMsg.ctlDesMsg.rArm_desTorMsg.assign(RARMDOF, 0.0);
    mdataSend.armTrajMsg.ctlDesMsg.lArm_MDP_desJntVel.assign(LARMDOF, 0.0);
    mdataSend.armTrajMsg.ctlDesMsg.rArm_MDP_desJntVel.assign(RARMDOF, 0.0);

    for(int i = 0; i < LARMDOF; i++){
        mdataSend.armTrajMsg.ctlDesMsg.lArm_rigidity_mod[i] = m_rigidityCtlMod;
        mdataSend.armTrajMsg.ctlDesMsg.rArm_rigidity_mod[i] = m_rigidityCtlMod;
    }

    //针对底盘控制进行赋值
    mdataSend.baseCtlMsg.chassis_cmd.linear_x = 0.0;
    mdataSend.baseCtlMsg.chassis_cmd.linear_y = 0.0;
    mdataSend.baseCtlMsg.chassis_cmd.angular_z = 0.0;

    mdataSend.baseCtlMsg.lifter_cmd.control_mod = 0;        //默认处于遥操作模式
    mdataSend.baseCtlMsg.lifter_cmd.move_cmd = 0;
    mdataSend.baseCtlMsg.lifter_cmd.target_position = 1;
    mdataSend.baseCtlMsg.lifter_cmd.save_preset = false;

    mdataSend.l_ifJntStab = lArm_ifJntStab_v2_1;
    mdataSend.r_ifJntStab = rArm_ifJntStab_v2_1;
}

DotOutpWrap allDemoTraj::TrajOutp_v2()
{
    //若不是第2个点，则应当配套输出第1个点的点位
    if(!ifSecondPoint_v2){

        if(mdataSend.armTrajMsg.ctlTransMod == 1){
            mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg = ctlDesMsg_v2_p1.desPos_l_JointSpace;
            mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg = ctlDesMsg_v2_p1.desPos_r_JointSpace;
        }else{
            mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg = QuaternionPos_trans(ctlDesMsg_v2_p1.desPos_l_Descartes);
            mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg = QuaternionPos_trans(ctlDesMsg_v2_p1.desPos_r_Descartes);
        }

        ifSecondPoint_v2 = true;
        return mdataSend;
    }else{
        //控制模式修改
        mdataSend.l_ifJntStab = lArm_ifJntStab_v2_2;
        mdataSend.r_ifJntStab = rArm_ifJntStab_v2_2;

        mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.ifUseTemp = 1;
        mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.JntTrajTime_L.assign(LARMDOF, 0.8);
        mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.JntTrajTime_R.assign(RARMDOF, 0.8);
        for(int i = 0; i < LARMDOF; i++){
            mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.lArm_stabMsg[i].ifUseStab = lArm_ifJntStab_v2_2[i];
            mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.rArm_stabMsg[i].ifUseStab = rArm_ifJntStab_v2_2[i];
        }

        //目标位置修改
        if(mdataSend.armTrajMsg.ctlTransMod == 1){
            mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg = ctlDesMsg_v2_p2.desPos_l_JointSpace;
            mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg = ctlDesMsg_v2_p2.desPos_r_JointSpace;
        }else{
            mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg = QuaternionPos_trans(ctlDesMsg_v2_p2.desPos_l_Descartes);
            mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg = QuaternionPos_trans(ctlDesMsg_v2_p2.desPos_r_Descartes);
        }

        return mdataSend;
    }
}

//----------轨迹3----------
//极限速度测试轨迹 -- 先抬起至起始位置，再开始运动

void allDemoTraj::init_v3()
{
    //连续轨迹计算函数

    //关节起始位置
    //关节空间
    std::vector<double> lArm_stJntPos, rArm_stJntPos;
    lArm_stJntPos = lArm_zeroPos;
    rArm_stJntPos = rArm_zeroPos;
    //笛卡尔空间
    std::vector<double> lArm_stDescPos, rArm_stDescPos;
    lArm_stDescPos = l_descZeroPos;
    rArm_stDescPos = r_descZeroPos;

    ctlStMsg_v3.desPos_l_Descartes = lArm_stDescPos;
    ctlStMsg_v3.desPos_r_Descartes = rArm_stDescPos;
    ctlStMsg_v3.desPos_l_JointSpace = lArm_stJntPos;
    ctlStMsg_v3.desPos_r_JointSpace = rArm_stJntPos;

    ctlStMsg_v3.desTor_l_Descartes.assign(6, 0.0);
    ctlStMsg_v3.desTor_r_Descartes.assign(6, 0.0);
    ctlStMsg_v3.desTor_l_JointSpace.assign(LARMDOF, 0.0);
    ctlStMsg_v3.desTor_r_JointSpace.assign(RARMDOF, 0.0);

    //关节目标位置1
    //关节空间的目标位置
    std::vector<double> lArm_desJntPos_1, rArm_desJntPos_1;
    lArm_desJntPos_1 = {
    };
    rArm_desJntPos_1 = {
        -1.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    };
    //笛卡尔空间目标位置
    std::vector<double> lArm_desDescPos_1, rArm_desDescPos_1;
    lArm_desDescPos_1 = {
        0.23, 0.255, -0.309, 0.0, 0.0, 0.0
    };
    rArm_desDescPos_1 = {
        0.23, -0.255, -0.309, 0.0, 0.0, 0.0
    };

    ctlDesMsg_v3_p1.desPos_l_Descartes = lArm_desDescPos_1;
    ctlDesMsg_v3_p1.desPos_r_Descartes = rArm_desDescPos_1;
    ctlDesMsg_v3_p1.desPos_l_JointSpace = lArm_desJntPos_1;
    ctlDesMsg_v3_p1.desPos_r_JointSpace = rArm_desJntPos_1;

    ctlDesMsg_v3_p1.desTor_l_Descartes.assign(6, 0.0);
    ctlDesMsg_v3_p1.desTor_r_Descartes.assign(6, 0.0);
    ctlDesMsg_v3_p1.desTor_l_JointSpace.assign(LARMDOF, 0.0);
    ctlDesMsg_v3_p1.desTor_r_JointSpace.assign(RARMDOF, 0.0);

    //角度输出模式是否为静止的
    lArm_ifJntStab_v3_1 = { 1, 1, 1, 1, 1, 1, 1 };
    rArm_ifJntStab_v3_1 = { 0, 1, 1, 1, 1, 1, 1 };

    //关节目标位置2
    //关节空间的目标位置
    std::vector<double> lArm_desJntPos_2, rArm_desJntPos_2;
    lArm_desJntPos_2 = {
        1.4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    };
    rArm_desJntPos_2 = {
        1.4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    };
    //笛卡尔空间目标位置
    std::vector<double> lArm_desDescPos_2, rArm_desDescPos_2;
    lArm_desDescPos_2 = {
        0.35, 0.255, -0.309, 0.0, 0.0, 0.0
    }; 
    rArm_desDescPos_2 = {
        0.35, -0.255, -0.309, 0.0, 0.0, 0.0
    };

    ctlDesMsg_v3_p2.desPos_l_Descartes = lArm_desDescPos_2;
    ctlDesMsg_v3_p2.desPos_r_Descartes = rArm_desDescPos_2;
    ctlDesMsg_v3_p2.desPos_l_JointSpace = lArm_desJntPos_2;
    ctlDesMsg_v3_p2.desPos_r_JointSpace = rArm_desJntPos_2;

    ctlDesMsg_v3_p2.desTor_l_Descartes.assign(6, 0.0);
    ctlDesMsg_v3_p2.desTor_r_Descartes.assign(6, 0.0);
    ctlDesMsg_v3_p2.desTor_l_JointSpace.assign(LARMDOF, 0.0);
    ctlDesMsg_v3_p2.desTor_r_JointSpace.assign(RARMDOF, 0.0);

    //角度输出模式是否为静止的
    lArm_ifJntStab_v3_2 = { 1, 1, 1, 1, 1, 1, 1 };
    rArm_ifJntStab_v3_2 = { 0, 1, 1, 1, 1, 1, 1 };

    double trajTime = 1.4;
    double accRate = 1.8;
    traj_time_v3 = 0.88;
    runinStabMsgWrap stabMsg;
    stabMsg.ifUseStab = true;
    stabMsg.start_stabTimes = 50;
    stabMsg.des_stabTimes = 50;

    //关节限位控制模式
    mdataSend.armTrajMsg.JntStabMod_l.ifUseJntLim = 0;
    mdataSend.armTrajMsg.JntStabMod_l.limJntNum = 1;
    mdataSend.armTrajMsg.JntStabMod_l.limJntID = {3};
    mdataSend.armTrajMsg.JntStabMod_l.limJntAng = {0.1};

    mdataSend.armTrajMsg.JntStabMod_r.ifUseJntLim = 0;
    mdataSend.armTrajMsg.JntStabMod_r.limJntNum = 1;
    mdataSend.armTrajMsg.JntStabMod_r.limJntID = {3};
    mdataSend.armTrajMsg.JntStabMod_r.limJntAng = {-0.1};

    //运动控制模式
    mdataSend.armTrajMsg.ctlTransMod = 1;
    mdataSend.armTrajMsg.TorFeedforwardCtl = 0;

    //控制量初始化
    mdataSend.armTrajMsg.ctlDesMsg.lArm_rigidity_mod.assign(7, 1);
    mdataSend.armTrajMsg.ctlDesMsg.rArm_rigidity_mod.assign(7, 1);
    mdataSend.armTrajMsg.ctlDesMsg.lArm_JntKp = lArm_defaultKp_low;
    mdataSend.armTrajMsg.ctlDesMsg.lArm_JntKd = lArm_defaultKd_low;
    mdataSend.armTrajMsg.ctlDesMsg.rArm_JntKp = rArm_defaultKp_low;
    mdataSend.armTrajMsg.ctlDesMsg.rArm_JntKd = rArm_defaultKd_low;

    //单点控制初始化
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.ifUseTemp = 0;         //是否使用循环输出

    double dotTrajTime = trajTime;                                               //轨迹时间2.0s
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.JntTrajTime_L.assign(LARMDOF, dotTrajTime);
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.JntTrajTime_R.assign(RARMDOF, dotTrajTime);
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.JntAccRate_L.assign(LARMDOF, accRate);
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.JntAccRate_R.assign(RARMDOF, accRate);

    //单点位静止消息控制
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.ifReDefineStabJnt = false;
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.lArm_stabMsg.assign(LARMDOF, stabMsg);
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.rArm_stabMsg.assign(RARMDOF, stabMsg);

    //控制量赋值
    mdataSend.armTrajMsg.ctlDesMsg.ifInitUseRet = 0;        //是否使用反馈值

    if(mdataSend.armTrajMsg.ctlTransMod == 1 || mdataSend.armTrajMsg.ctlTransMod == 3){
        mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg = lArm_stJntPos;
        mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg = rArm_stJntPos;
    }else{

        Pose lArm_Quaternion_pos, rArm_Quaternion_pos;
        lArm_Quaternion_pos = rpyToQuaternion(
            lArm_stDescPos[0], lArm_stDescPos[1], lArm_stDescPos[2], 
            lArm_stDescPos[3], lArm_stDescPos[4], lArm_stDescPos[5]
        );
        rArm_Quaternion_pos = rpyToQuaternion(
            rArm_stDescPos[0], rArm_stDescPos[1], rArm_stDescPos[2],
            rArm_stDescPos[3], rArm_stDescPos[4], rArm_stDescPos[5]
        );

        std::vector<double> lArm_desDescQuaPos, rArm_desDescQuaPos;
        lArm_desDescQuaPos = {
            lArm_Quaternion_pos.x, lArm_Quaternion_pos.y, lArm_Quaternion_pos.z,
            lArm_Quaternion_pos.q.w, lArm_Quaternion_pos.q.x, lArm_Quaternion_pos.q.y, lArm_Quaternion_pos.q.z
        };
        rArm_desDescQuaPos = {
            rArm_Quaternion_pos.x, rArm_Quaternion_pos.y, rArm_Quaternion_pos.z,
            rArm_Quaternion_pos.q.w, rArm_Quaternion_pos.q.x, rArm_Quaternion_pos.q.y, rArm_Quaternion_pos.q.z
        };

        mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg = lArm_desDescQuaPos;
        mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg = rArm_desDescQuaPos;
    }
    mdataSend.armTrajMsg.ctlDesMsg.lArm_desTorMsg.assign(LARMDOF, 0.0);
    mdataSend.armTrajMsg.ctlDesMsg.rArm_desTorMsg.assign(RARMDOF, 0.0);
    mdataSend.armTrajMsg.ctlDesMsg.lArm_MDP_desJntVel.assign(LARMDOF, 0.0);
    mdataSend.armTrajMsg.ctlDesMsg.rArm_MDP_desJntVel.assign(RARMDOF, 0.0);

    for(int i = 0; i < LARMDOF; i++){
        mdataSend.armTrajMsg.ctlDesMsg.lArm_rigidity_mod[i] = m_rigidityCtlMod;
        mdataSend.armTrajMsg.ctlDesMsg.rArm_rigidity_mod[i] = m_rigidityCtlMod;
    }

    //针对底盘控制进行赋值
    mdataSend.baseCtlMsg.chassis_cmd.linear_x = 0.0;
    mdataSend.baseCtlMsg.chassis_cmd.linear_y = 0.0;
    mdataSend.baseCtlMsg.chassis_cmd.angular_z = 0.0;

    mdataSend.baseCtlMsg.lifter_cmd.control_mod = 0;        //默认处于遥操作模式
    mdataSend.baseCtlMsg.lifter_cmd.move_cmd = 0;
    mdataSend.baseCtlMsg.lifter_cmd.target_position = 1;
    mdataSend.baseCtlMsg.lifter_cmd.save_preset = false; 

    mdataSend.l_ifJntStab = lArm_ifJntStab_v3_1;
    mdataSend.r_ifJntStab = rArm_ifJntStab_v3_1;

    //控制变量初始化
    ifSecondPoint_v3 = false;
}

DotOutpWrap allDemoTraj::TrajOutp_v3()
{
    //若不是第2个点，则应当配套输出第1个点的点位
    if(!ifSecondPoint_v3){
        ifSecondPoint_v3 = true;
        if(mdataSend.armTrajMsg.ctlTransMod == 1 || mdataSend.armTrajMsg.ctlTransMod == 3){
            mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg = ctlDesMsg_v3_p1.desPos_l_JointSpace;
            mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg = ctlDesMsg_v3_p1.desPos_r_JointSpace;
        }else{
            mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg = QuaternionPos_trans(ctlDesMsg_v3_p1.desPos_l_Descartes);
            mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg = QuaternionPos_trans(ctlDesMsg_v3_p1.desPos_r_Descartes);
        }
        return mdataSend;
    }else{
        //控制参量修改
        mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.ifUseTemp = true;
        mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.JntTrajTime_L.assign(LARMDOF, traj_time_v3);
        mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.JntTrajTime_R.assign(RARMDOF, traj_time_v3);

        mdataSend.l_ifJntStab = lArm_ifJntStab_v3_2;
        mdataSend.r_ifJntStab = rArm_ifJntStab_v3_2;

        mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.ifReDefineStabJnt;

        //目标位置修改
        if(mdataSend.armTrajMsg.ctlTransMod == 1){
            mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg = ctlDesMsg_v3_p2.desPos_l_JointSpace;
            mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg = ctlDesMsg_v3_p2.desPos_r_JointSpace;
        }else{
            mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg = QuaternionPos_trans(ctlDesMsg_v3_p2.desPos_l_Descartes);
            mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg = QuaternionPos_trans(ctlDesMsg_v3_p2.desPos_r_Descartes);
        }

        //重新初始化
        if(ifRedefineStabJntAng){
            mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.ifReDefineStabJnt = true;
            ifRedefineStabJntAng = false;
        }else{
            mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.ifReDefineStabJnt = false;
        }

        return mdataSend;
    }
}


//----------轨迹4----------
//轨迹控制轨迹

void allDemoTraj::init_v4()
{
    //连续轨迹计算函数

    //插补器初始化：
    lArm_DescRuniner_v4.resize(6);
    rArm_DescRuniner_v4.resize(6);

    lArm_JntRuniner_v4.resize(LARMDOF);
    rArm_JntRuniner_v4.resize(RARMDOF);

    for(int i = 0; i < 6; i++){
        lArm_DescRuniner_v4[i].Init();
        rArm_DescRuniner_v4[i].Init();
    }
    for(int i = 0; i < LARMDOF; i++){
        lArm_JntRuniner_v4[i].Init();
        rArm_JntRuniner_v4[i].Init();
    }

    //---------------始末位置创建---------------
    //关节起始位置
    //关节空间
    std::vector<double> lArm_stJntPos, rArm_stJntPos;
    lArm_stJntPos = lArm_zeroPos;
    rArm_stJntPos = rArm_zeroPos;
    //笛卡尔空间
    std::vector<double> lArm_stDescPos, rArm_stDescPos;
    lArm_stDescPos = l_descZeroPos;
    rArm_stDescPos = r_descZeroPos;

    ctlStMsg_v4.desPos_l_Descartes = lArm_stDescPos;
    ctlStMsg_v4.desPos_r_Descartes = rArm_stDescPos;
    ctlStMsg_v4.desPos_l_JointSpace = lArm_stJntPos;
    ctlStMsg_v4.desPos_r_JointSpace = rArm_stJntPos;

    ctlStMsg_v4.desTor_l_Descartes.assign(6, 0.0);
    ctlStMsg_v4.desTor_r_Descartes.assign(6, 0.0);
    ctlStMsg_v4.desTor_l_JointSpace.assign(LARMDOF, 0.0);
    ctlStMsg_v4.desTor_r_JointSpace.assign(RARMDOF, 0.0);

    //关节目标位置
    //关节空间的目标位置
    std::vector<double> lArm_desJntPos, rArm_desJntPos;
    lArm_desJntPos = {
        0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5
    };
    rArm_desJntPos = {
        0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5
    };
    //笛卡尔空间目标位置
    std::vector<double> lArm_desDescPos, rArm_desDescPos;
    lArm_desDescPos = {
        0.3, 0.144998, -0.16, -1.7, -0.8, 0.0 
    };
    rArm_desDescPos = {
        0.3, -0.144998, -0.16, 1.7, -0.8, 0.0
    };

    ctlDesMsg_v4.desPos_l_Descartes = lArm_desDescPos;
    ctlDesMsg_v4.desPos_r_Descartes = rArm_desDescPos;
    ctlDesMsg_v4.desPos_l_JointSpace = lArm_desJntPos;
    ctlDesMsg_v4.desPos_r_JointSpace = rArm_desJntPos;

    ctlDesMsg_v4.desTor_l_Descartes.assign(6, 0.0);
    ctlDesMsg_v4.desTor_r_Descartes.assign(6, 0.0);
    ctlDesMsg_v4.desTor_l_JointSpace.assign(LARMDOF, 0.0);
    ctlDesMsg_v4.desTor_r_JointSpace.assign(RARMDOF, 0.0);

    //---------------运动轨迹创建---------------

    //trajLen = 200;
    trajLen = 200;
    temp_times = 0;

    double m_trajTime = 0.32;
    double m_accRate = 1.2;
    int st_len = 50;    //再初始阶段停留时间
    int des_len = 50;   //再目标位置停留时间

    //--------------轨迹计数器对象创建---------------
    counter_1 = 0;
    counter_2 = 0;
    all_count = 3;
    ifReachDesPos = false;

    //创建轨迹
    std::vector<ctlDesMsgWrap> ctlDesTraj_f;

    std::vector<std::vector<double>> lArm_DescPosTraj;
    std::vector<std::vector<double>> rArm_DescPosTraj;

    std::vector<std::vector<double>> lArm_JntPosTraj;
    std::vector<std::vector<double>> rArm_JntPosTraj;

    lArm_DescPosTraj.resize(6);
    rArm_DescPosTraj.resize(6);

    lArm_JntPosTraj.resize(LARMDOF);
    rArm_JntPosTraj.resize(RARMDOF);

    for(int i = 0; i < 6; i++){
        TrajResult temp_trajRes_l = lArm_DescRuniner_v4[i].dPos_queue_stabTime_dot((lArm_desDescPos[i] - lArm_stDescPos[i]), m_trajTime, m_accRate);
        TrajResult temp_trajRes_r = rArm_DescRuniner_v4[i].dPos_queue_stabTime_dot((rArm_desDescPos[i] - rArm_stDescPos[i]), m_trajTime, m_accRate);

        for(int j = 0; j < temp_trajRes_l.positionIncrements.size(); j++){
            lArm_DescPosTraj[i].push_back(temp_trajRes_l.positionIncrements[j] + lArm_stDescPos[i]);
        }
        for(int j = 0; j < temp_trajRes_r.positionIncrements.size(); j++){
            rArm_DescPosTraj[i].push_back(temp_trajRes_r.positionIncrements[j] + rArm_stDescPos[i]);
        }
    }

    for(int i = 0; i < LARMDOF; i++){
        TrajResult temp_trajRes_l = lArm_JntRuniner_v4[i].dPos_queue_stabTime_dot((lArm_desJntPos[i] - lArm_stJntPos[i]), m_trajTime, m_accRate);
        TrajResult temp_trajRes_r = rArm_JntRuniner_v4[i].dPos_queue_stabTime_dot((rArm_desJntPos[i] - rArm_stJntPos[i]), m_trajTime, m_accRate);

        for(int j = 0; j < temp_trajRes_l.positionIncrements.size(); j++){
            lArm_JntPosTraj[i].push_back(temp_trajRes_l.positionIncrements[j] + lArm_stJntPos[i]);
        }
        for(int j = 0; j < temp_trajRes_r.positionIncrements.size(); j++){
            rArm_JntPosTraj[i].push_back(temp_trajRes_r.positionIncrements[j] + rArm_stJntPos[i]);
        }
    }

    for(int i = 0; i < lArm_DescPosTraj[0].size(); i++){
        ctlDesMsgWrap temp_ctlDesMsg;

        temp_ctlDesMsg.desPos_l_Descartes.assign(6, 0.0);
        temp_ctlDesMsg.desPos_r_Descartes.assign(6, 0.0);
        temp_ctlDesMsg.desTor_l_Descartes.assign(6, 0.0);
        temp_ctlDesMsg.desTor_r_Descartes.assign(6, 0.0);

        temp_ctlDesMsg.desPos_l_JointSpace.assign(LARMDOF, 0.0);
        temp_ctlDesMsg.desPos_r_JointSpace.assign(RARMDOF, 0.0);
        temp_ctlDesMsg.desTor_l_JointSpace.assign(LARMDOF, 0.0);
        temp_ctlDesMsg.desTor_r_JointSpace.assign(RARMDOF, 0.0);

        for(int j = 0; j < 6; j++){
            temp_ctlDesMsg.desPos_l_Descartes[j] = lArm_DescPosTraj[j][i];
            temp_ctlDesMsg.desPos_r_Descartes[j] = rArm_DescPosTraj[j][i];    
        }

        for(int j = 0; j < LARMDOF; j++){
            temp_ctlDesMsg.desPos_l_JointSpace[j] = lArm_JntPosTraj[j][i];
            temp_ctlDesMsg.desPos_r_JointSpace[j] = rArm_JntPosTraj[j][i];
        }

        ctlDesTraj_f.push_back(temp_ctlDesMsg);
    }


    std::vector<ctlDesMsgWrap> ctlDesTraj_b(ctlDesTraj_f.rbegin(), ctlDesTraj_f.rend());

    //生成总轨迹
    for(int i = 0; i < st_len; i++){
        ctlDesTraj_v4.push_back(ctlStMsg_v4);
    }
    for(int i = 0; i < ctlDesTraj_f.size(); i++){
        ctlDesTraj_v4.push_back(ctlDesTraj_f[i]);
    }
    for(int i = 0; i < des_len; i++){
        ctlDesTraj_v4.push_back(ctlDesMsg_v4);
    }
    for(int i = 0; i < ctlDesTraj_b.size(); i++){
        ctlDesTraj_v4.push_back(ctlDesTraj_b[i]);
    }
    ctlDesTraj_v4.push_back(ctlStMsg_v4);
    all_len = ctlDesTraj_v4.size();

    counter_1 = st_len + ctlDesTraj_f.size() + 5;

    //----------------------------------------

    //角度输出模式是否为静止的
    std::vector<int> lArm_ifJntStab, rArm_ifJntStab;
    lArm_ifJntStab = { 0, 0, 0, 0, 0, 0, 0 };
    rArm_ifJntStab = { 0, 0, 0, 0, 0, 0, 0 };

    //关节限位控制模式
    mdataSend.armTrajMsg.JntStabMod_l.ifUseJntLim = 0;
    mdataSend.armTrajMsg.JntStabMod_l.limJntNum = 1;
    mdataSend.armTrajMsg.JntStabMod_l.limJntID = {3};
    mdataSend.armTrajMsg.JntStabMod_l.limJntAng = {0.1};

    mdataSend.armTrajMsg.JntStabMod_r.ifUseJntLim = 0;
    mdataSend.armTrajMsg.JntStabMod_r.limJntNum = 1;
    mdataSend.armTrajMsg.JntStabMod_r.limJntID = {3};
    mdataSend.armTrajMsg.JntStabMod_r.limJntAng = {-0.1};

    //运动控制模式
    mdataSend.armTrajMsg.ctlTransMod = 4;
    mdataSend.armTrajMsg.TorFeedforwardCtl = 0;

    //控制量初始化
    mdataSend.armTrajMsg.ctlDesMsg.lArm_rigidity_mod.assign(7, 1);
    mdataSend.armTrajMsg.ctlDesMsg.rArm_rigidity_mod.assign(7, 1);
    mdataSend.armTrajMsg.ctlDesMsg.lArm_JntKp = lArm_defaultKp_low;
    mdataSend.armTrajMsg.ctlDesMsg.lArm_JntKd = lArm_defaultKd_low;
    mdataSend.armTrajMsg.ctlDesMsg.rArm_JntKp = rArm_defaultKp_low;
    mdataSend.armTrajMsg.ctlDesMsg.rArm_JntKd = rArm_defaultKd_low;

    //单点控制初始化
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.ifUseTemp = 0;         //是否使用循环输出

    double dotTrajTime = 0.6;                                               //轨迹时间2.0s
    double accRate = 1.2;                                                   //加速时间占比
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.JntTrajTime_L.assign(LARMDOF, dotTrajTime);
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.JntTrajTime_R.assign(RARMDOF, dotTrajTime);
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.JntAccRate_L.assign(LARMDOF, accRate);
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.JntAccRate_R.assign(RARMDOF, accRate);

    //单点位静止消息控制
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.ifReDefineStabJnt = false;
    runinStabMsgWrap runin_stab_msg;
    runin_stab_msg.ifUseStab = true;
    runin_stab_msg.start_stabTimes = 50;
    runin_stab_msg.des_stabTimes = 50;
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.lArm_stabMsg.assign(LARMDOF, runin_stab_msg);
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.rArm_stabMsg.assign(RARMDOF, runin_stab_msg);

    //控制量赋值
    mdataSend.armTrajMsg.ctlDesMsg.ifInitUseRet = 0;        //是否使用反馈值

    if(mdataSend.armTrajMsg.ctlTransMod == 1 || mdataSend.armTrajMsg.ctlTransMod == 3){
        mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg = lArm_stJntPos;
        mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg = rArm_stJntPos;
    }else{
        mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg = lArm_stDescPos;
        mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg = rArm_stDescPos;
    }

    mdataSend.armTrajMsg.ctlDesMsg.lArm_desTorMsg.assign(LARMDOF, 0.0);
    mdataSend.armTrajMsg.ctlDesMsg.rArm_desTorMsg.assign(RARMDOF, 0.0);
    mdataSend.armTrajMsg.ctlDesMsg.lArm_MDP_desJntVel.assign(LARMDOF, 0.0);
    mdataSend.armTrajMsg.ctlDesMsg.rArm_MDP_desJntVel.assign(RARMDOF, 0.0);

    for(int i = 0; i < LARMDOF; i++){
        mdataSend.armTrajMsg.ctlDesMsg.lArm_rigidity_mod[i] = m_rigidityCtlMod;
        mdataSend.armTrajMsg.ctlDesMsg.rArm_rigidity_mod[i] = m_rigidityCtlMod;
    }

    //针对底盘控制进行赋值
    mdataSend.baseCtlMsg.chassis_cmd.linear_x = 0.0;
    mdataSend.baseCtlMsg.chassis_cmd.linear_y = 0.0;
    mdataSend.baseCtlMsg.chassis_cmd.angular_z = 0.0;

    mdataSend.baseCtlMsg.lifter_cmd.control_mod = 0;        //默认处于遥操作模式
    mdataSend.baseCtlMsg.lifter_cmd.move_cmd = 0;
    mdataSend.baseCtlMsg.lifter_cmd.target_position = 1;
    mdataSend.baseCtlMsg.lifter_cmd.save_preset = false;    

    mdataSend.l_ifJntStab = lArm_ifJntStab;
    mdataSend.r_ifJntStab = rArm_ifJntStab;
}

DotOutpWrap allDemoTraj::TrajOutp_v4()
{
    if(counter_2 < all_count){
        if(mdataSend.armTrajMsg.ctlTransMod == 1 || mdataSend.armTrajMsg.ctlTransMod == 3){
            mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg = ctlDesTraj_v4[temp_times].desPos_l_JointSpace;
            mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg = ctlDesTraj_v4[temp_times].desPos_r_JointSpace;
        }else{
            mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg = ctlDesTraj_v4[temp_times].desPos_l_Descartes;
            mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg = ctlDesTraj_v4[temp_times].desPos_r_Descartes;
        }

        temp_times += 1;
        if(temp_times >= all_len){
            temp_times = 0;
            counter_2 += 1;
        }

    }else{
        //先运动到目标位置
        if(!ifReachDesPos){
            if(mdataSend.armTrajMsg.ctlTransMod == 1 || mdataSend.armTrajMsg.ctlTransMod == 3){
                mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg = ctlDesTraj_v4[temp_times].desPos_l_JointSpace;
                mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg = ctlDesTraj_v4[temp_times].desPos_r_JointSpace;
            }else{
                mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg = ctlDesTraj_v4[temp_times].desPos_l_Descartes;
                mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg = ctlDesTraj_v4[temp_times].desPos_r_Descartes;
            }

            std::cout << "temp times = " << temp_times << " || " << counter_1 << std::endl;
            if(temp_times >= counter_1){
                if(mdataSend.armTrajMsg.ctlTransMod == 1 || mdataSend.armTrajMsg.ctlTransMod == 3){
                    mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg = ctlDesTraj_v4[temp_times].desPos_l_JointSpace;
                    mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg = ctlDesTraj_v4[temp_times].desPos_r_JointSpace;
                }else{
                    mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg = ctlDesMsg_v4.desPos_l_Descartes;
                    mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg = ctlDesMsg_v4.desPos_r_Descartes;
                }
                m_rigidityCtlMod = 1;
                for(int k = 0; k < 7; k++){
                    mdataSend.armTrajMsg.ctlDesMsg.lArm_rigidity_mod[k] = m_rigidityCtlMod;
                    mdataSend.armTrajMsg.ctlDesMsg.rArm_rigidity_mod[k] = m_rigidityCtlMod;
                }
            }else{
                temp_times += 1;
            }
        }
    }

    return mdataSend;
}

//----------轨迹5----------
//零空间控制轨迹

void allDemoTraj::init_v5()
{
    //连续轨迹计算函数

    //关节起始位置
    //关节空间
    std::vector<double> lArm_stJntPos, rArm_stJntPos;
    lArm_stJntPos = lArm_zeroPos;
    rArm_stJntPos = rArm_zeroPos;
    //笛卡尔空间
    std::vector<double> lArm_stDescPos, rArm_stDescPos;
    lArm_stDescPos = l_descZeroPos;
    rArm_stDescPos = r_descZeroPos;

    ctlStMsg_v5.desPos_l_Descartes = lArm_stDescPos;
    ctlStMsg_v5.desPos_r_Descartes = rArm_stDescPos;
    ctlStMsg_v5.desPos_l_JointSpace = lArm_stJntPos;
    ctlStMsg_v5.desPos_r_JointSpace = rArm_stJntPos;

    ctlStMsg_v5.desTor_l_Descartes.assign(6, 0.0);
    ctlStMsg_v5.desTor_r_Descartes.assign(6, 0.0);
    ctlStMsg_v5.desTor_l_JointSpace.assign(LARMDOF, 0.0);
    ctlStMsg_v5.desTor_r_JointSpace.assign(RARMDOF, 0.0);

    //关节目标位置
    //关节空间的目标位置
    std::vector<double> lArm_desJntPos, rArm_desJntPos;
    lArm_desJntPos = {
        0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5
    };
    rArm_desJntPos = {
        0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5
    };
    //笛卡尔空间目标位置
    std::vector<double> lArm_desDescPos, rArm_desDescPos;
    lArm_desDescPos = {
        0.3, 0.204998, -0.16, -0.8, -0.8, 0.0 
    };
    rArm_desDescPos = {
        0.3, -0.204998, -0.16, 0.8, -0.8, 0.0
    };

    ctlDesMsg_v5.desPos_l_Descartes = lArm_desDescPos;
    ctlDesMsg_v5.desPos_r_Descartes = rArm_desDescPos;
    ctlDesMsg_v5.desPos_l_JointSpace = lArm_desJntPos;
    ctlDesMsg_v5.desPos_r_JointSpace = rArm_desJntPos;

    ctlDesMsg_v5.desTor_l_Descartes.assign(6, 0.0);
    ctlDesMsg_v5.desTor_r_Descartes.assign(6, 0.0);
    ctlDesMsg_v5.desTor_l_JointSpace.assign(LARMDOF, 0.0);
    ctlDesMsg_v5.desTor_r_JointSpace.assign(RARMDOF, 0.0);

    //角度输出模式是否为静止的
    std::vector<int> lArm_ifJntStab, rArm_ifJntStab;
    lArm_ifJntStab = { 0, 0, 0, 0, 0, 0, 0  };
    rArm_ifJntStab = { 0, 0, 0, 0, 0, 0, 0  };

    //---------------零空间控制轨迹定义--------------

    //信息定义：
    double l_limJntAng_J3_st, l_limJntAng_J3_end;
    double r_limJntAng_J3_st, r_limJntAng_J3_end;
    l_limJntAng_J3_st = 0.0;
    l_limJntAng_J3_end = 0.8;
    r_limJntAng_J3_st = 0.0;
    r_limJntAng_J3_end = -0.8;

    trajLen_v5 = 100;
    temp_times_v5 = 0;

    //始末阶段静止轮数
    int stab_st = 50;
    int stab_end = 50;

    //始末位置生成：
    stJntLimMsg_l.ifUseJntLim = true;
    stJntLimMsg_l.limJntNum = 1;
    stJntLimMsg_l.limJntAng.resize(stJntLimMsg_l.limJntNum);
    stJntLimMsg_l.limJntLabel.resize(stJntLimMsg_l.limJntNum);
    stJntLimMsg_l.limJntLabel[0] = "l_joint_3";
    stJntLimMsg_l.limJntAng[0] = l_limJntAng_J3_st;

    endJntLimMsg_l.ifUseJntLim = stJntLimMsg_l.ifUseJntLim;
    endJntLimMsg_l.limJntNum = stJntLimMsg_l.limJntNum;
    endJntLimMsg_l.limJntLabel.resize(endJntLimMsg_l.limJntNum);
    endJntLimMsg_l.limJntAng.resize(endJntLimMsg_l.limJntNum);
    endJntLimMsg_l.limJntLabel[0] = stJntLimMsg_l.limJntLabel[0];
    endJntLimMsg_l.limJntAng[0] = l_limJntAng_J3_end;

    stJntLimMsg_r.ifUseJntLim = true;
    stJntLimMsg_r.limJntNum = 1;
    stJntLimMsg_r.limJntAng.resize(stJntLimMsg_r.limJntNum);
    stJntLimMsg_r.limJntLabel.resize(stJntLimMsg_r.limJntNum);
    stJntLimMsg_r.limJntLabel[0] = "r_joint_3";
    stJntLimMsg_r.limJntAng[0] = r_limJntAng_J3_st;

    endJntLimMsg_r.ifUseJntLim = stJntLimMsg_r.ifUseJntLim;
    endJntLimMsg_r.limJntNum = stJntLimMsg_r.limJntNum;
    endJntLimMsg_r.limJntLabel.resize(endJntLimMsg_r.limJntNum);
    endJntLimMsg_r.limJntAng.resize(endJntLimMsg_r.limJntNum);
    endJntLimMsg_r.limJntLabel[0] = stJntLimMsg_r.limJntLabel[0];
    endJntLimMsg_r.limJntAng[0] = r_limJntAng_J3_end;

    //零空间限制轨迹输出
    std::vector<JntLimMsgWrap> lArmJntLimMsg_f, rArmJntLimMsg_f;
    std::vector<JntLimMsgWrap> lArmJntLimMsg_b, rArmJntLimMsg_b; 

    double derLimJntAng_l, derLimJntAng_r;
    derLimJntAng_l = (l_limJntAng_J3_end - l_limJntAng_J3_st)/trajLen_v5;
    derLimJntAng_r = (r_limJntAng_J3_end - r_limJntAng_J3_st)/trajLen_v5;
    for(int i = 0; i < trajLen_v5; i++){
        JntLimMsgWrap temp_l_JntLimMsg, temp_r_JntLimMsg;

        //设定本轮左臂关节限制信息
        temp_l_JntLimMsg.ifUseJntLim = stJntLimMsg_l.ifUseJntLim;
        temp_l_JntLimMsg.limJntNum = stJntLimMsg_l.limJntNum;
        temp_l_JntLimMsg.limJntAng.resize(temp_l_JntLimMsg.limJntNum);
        temp_l_JntLimMsg.limJntLabel.resize(temp_l_JntLimMsg.limJntNum);
        temp_l_JntLimMsg.limJntLabel[0] = stJntLimMsg_l.limJntLabel[0];
        temp_l_JntLimMsg.limJntAng[0] = l_limJntAng_J3_st + (i+1)*derLimJntAng_l;

        //设定本轮右臂关节限制信息
        temp_r_JntLimMsg.ifUseJntLim = stJntLimMsg_r.ifUseJntLim;
        temp_r_JntLimMsg.limJntNum = stJntLimMsg_r.limJntNum;
        temp_r_JntLimMsg.limJntAng.resize(temp_r_JntLimMsg.limJntNum);
        temp_r_JntLimMsg.limJntLabel.resize(temp_r_JntLimMsg.limJntNum);
        temp_r_JntLimMsg.limJntLabel[0] = stJntLimMsg_r.limJntLabel[0];
        temp_r_JntLimMsg.limJntAng[0] = r_limJntAng_J3_st + (i+1)*derLimJntAng_r;

        //装入轨迹
        lArmJntLimMsg_f.push_back(temp_l_JntLimMsg);
        rArmJntLimMsg_f.push_back(temp_r_JntLimMsg);
    }

    //返回轨迹生成
    lArmJntLimMsg_b.assign(lArmJntLimMsg_f.rbegin(), lArmJntLimMsg_f.rend());
    rArmJntLimMsg_b.assign(rArmJntLimMsg_f.rbegin(), rArmJntLimMsg_f.rend());

    //整体轨迹生成
    for(int i = 0; i < stab_st; i++){
        lArmJntLimMsg.push_back(stJntLimMsg_l);
        rArmJntLimMsg.push_back(stJntLimMsg_r);
    }
    for(int i = 0; i < trajLen_v5; i++){
        lArmJntLimMsg.push_back(lArmJntLimMsg_f[i]);
        rArmJntLimMsg.push_back(rArmJntLimMsg_f[i]);
    }
    for(int i = 0; i < stab_end; i++){
        lArmJntLimMsg.push_back(endJntLimMsg_l);
        rArmJntLimMsg.push_back(endJntLimMsg_r);
    }
    for(int i = 0; i < trajLen_v5; i++){
        lArmJntLimMsg.push_back(lArmJntLimMsg_b[i]);
        rArmJntLimMsg.push_back(rArmJntLimMsg_b[i]);
    }
    lArmJntLimMsg.push_back(stJntLimMsg_l);
    rArmJntLimMsg.push_back(stJntLimMsg_r);

    //左右两侧轨迹长度是一样的
    all_len_v5 = lArmJntLimMsg.size();

    //--------------------------------------------
    //
    double trajTime = 1.4;
    double accRate = 1.8;
    runinStabMsgWrap stabMsg;
    stabMsg.ifUseStab = true;
    stabMsg.start_stabTimes = 50;
    stabMsg.des_stabTimes = 50;

    //关节限位控制模式
    mdataSend.armTrajMsg.JntStabMod_l.ifUseJntLim = 1;
    mdataSend.armTrajMsg.JntStabMod_l.limJntNum = 1;
    mdataSend.armTrajMsg.JntStabMod_l.limJntID = {3};
    mdataSend.armTrajMsg.JntStabMod_l.limJntAng = stJntLimMsg_l.limJntAng;

    mdataSend.armTrajMsg.JntStabMod_r.ifUseJntLim = 1;
    mdataSend.armTrajMsg.JntStabMod_r.limJntNum = 1;
    mdataSend.armTrajMsg.JntStabMod_r.limJntID = {10};
    mdataSend.armTrajMsg.JntStabMod_r.limJntAng = stJntLimMsg_r.limJntAng;

    //运动控制模式
    mdataSend.armTrajMsg.ctlTransMod = 2;
    mdataSend.armTrajMsg.TorFeedforwardCtl = 0;

    //控制量初始化
    mdataSend.armTrajMsg.ctlDesMsg.lArm_rigidity_mod.assign(7, 1);
    mdataSend.armTrajMsg.ctlDesMsg.rArm_rigidity_mod.assign(7, 1);
    mdataSend.armTrajMsg.ctlDesMsg.lArm_JntKp = lArm_defaultKp_low;
    mdataSend.armTrajMsg.ctlDesMsg.lArm_JntKd = lArm_defaultKd_low;
    mdataSend.armTrajMsg.ctlDesMsg.rArm_JntKp = rArm_defaultKp_low;
    mdataSend.armTrajMsg.ctlDesMsg.rArm_JntKd = rArm_defaultKd_low;

    //单点控制初始化
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.ifUseTemp = 0;         //是否使用循环输出

    double dotTrajTime = trajTime;                                               //轨迹时间2.0s
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.JntTrajTime_L.assign(LARMDOF, dotTrajTime);
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.JntTrajTime_R.assign(RARMDOF, dotTrajTime);
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.JntAccRate_L.assign(LARMDOF, accRate);
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.JntAccRate_R.assign(RARMDOF, accRate);

    //单点位静止消息控制
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.ifReDefineStabJnt = false;
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.lArm_stabMsg.assign(LARMDOF, stabMsg);
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.rArm_stabMsg.assign(RARMDOF, stabMsg);

    //控制量赋值
    mdataSend.armTrajMsg.ctlDesMsg.ifInitUseRet = 0;        //是否使用反馈值

    for(int i = 0; i < LARMDOF; i++){
        mdataSend.armTrajMsg.ctlDesMsg.lArm_rigidity_mod[i] = m_rigidityCtlMod;
        mdataSend.armTrajMsg.ctlDesMsg.rArm_rigidity_mod[i] = m_rigidityCtlMod;
    }

    //针对底盘控制进行赋值
    mdataSend.baseCtlMsg.chassis_cmd.linear_x = 0.0;
    mdataSend.baseCtlMsg.chassis_cmd.linear_y = 0.0;
    mdataSend.baseCtlMsg.chassis_cmd.angular_z = 0.0;

    mdataSend.baseCtlMsg.lifter_cmd.control_mod = 0;        //默认处于遥操作模式
    mdataSend.baseCtlMsg.lifter_cmd.move_cmd = 0;
    mdataSend.baseCtlMsg.lifter_cmd.target_position = 1;
    mdataSend.baseCtlMsg.lifter_cmd.save_preset = false; 

    mdataSend.l_ifJntStab = lArm_ifJntStab;
    mdataSend.r_ifJntStab = rArm_ifJntStab;

    //初始化目标位置
    if(mdataSend.armTrajMsg.ctlTransMod == 1 || mdataSend.armTrajMsg.ctlTransMod == 3){
        mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg = ctlStMsg_v5.desPos_l_JointSpace;
        mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg = ctlStMsg_v5.desPos_r_JointSpace;
    }else{
        mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg = ctlStMsg_v5.desPos_l_Descartes;
        mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg = ctlStMsg_v5.desPos_r_Descartes;
    }

    mdataSend.armTrajMsg.ctlDesMsg.lArm_desTorMsg.assign(LARMDOF, 0.0);
    mdataSend.armTrajMsg.ctlDesMsg.rArm_desTorMsg.assign(RARMDOF, 0.0);
    mdataSend.armTrajMsg.ctlDesMsg.lArm_MDP_desJntVel.assign(LARMDOF, 0.0);
    mdataSend.armTrajMsg.ctlDesMsg.rArm_MDP_desJntVel.assign(RARMDOF, 0.0);
}

DotOutpWrap allDemoTraj::TrajOutp_v5(){

    if(!ifReachStartPos_v5){
        //进入到去往初始位置的控制流程

        if(mdataSend.armTrajMsg.ctlTransMod == 1 || mdataSend.armTrajMsg.ctlTransMod == 3){
            mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg = ctlDesMsg_v5.desPos_l_JointSpace;
            mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg = ctlDesMsg_v5.desPos_r_JointSpace;
        }else{
            mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg = ctlDesMsg_v5.desPos_l_Descartes;
            mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg = ctlDesMsg_v5.desPos_r_Descartes;
        }
        ifReachStartPos_v5 = true;

        return mdataSend;
    }else{
        //进入零空间控制模式
        mdataSend.armTrajMsg.ctlTransMod = 4;

        DotOutpWrap temp_dataSend;
        temp_dataSend = mdataSend;
        
        temp_dataSend.armTrajMsg.JntStabMod_l.ifUseJntLim = 1;
        temp_dataSend.armTrajMsg.JntStabMod_l.limJntAng = lArmJntLimMsg[temp_times_v5].limJntAng;
        temp_dataSend.armTrajMsg.JntStabMod_r.ifUseJntLim = 1;
        temp_dataSend.armTrajMsg.JntStabMod_r.limJntAng = rArmJntLimMsg[temp_times_v5].limJntAng;

        //更新迭代次数
        temp_times_v5 += 1;
        if(temp_times_v5 >= all_len_v5){
            temp_times_v5 = 0;
        }
        return temp_dataSend;
    }
}

void allDemoTraj::init_v6(){ 

    std::cout << "请输入选择的点编号：0～2" << std::endl;
    std::cin >> point_choose_v6;
    if(point_choose_v6 < 0){
        point_choose_v6 = 0;
    }
    if(point_choose_v6 > 2){
        point_choose_v6 = 2;
    }

    //求解器初始化
    lArm_DescRuniner_v6.resize(6);
    rArm_DescRuniner_v6.resize(6);

    lArm_JntRuniner_v6.resize(LARMDOF);
    rArm_JntRuniner_v6.resize(RARMDOF);

    for(int i = 0; i < 6; i++){
        lArm_DescRuniner_v6[i].Init();
        rArm_DescRuniner_v6[i].Init();
    }
    for(int i = 0; i < LARMDOF; i++){
        lArm_JntRuniner_v6[i].Init();
        rArm_JntRuniner_v6[i].Init();
    }

    //连续轨迹计算函数
    temp_times_v6 = 0;
    dotID_v6 = 0;
    ifFw = true;

    //关节起始位置
    //关节空间
    std::vector<double> lArm_stJntPos, rArm_stJntPos;
    lArm_stJntPos = lArm_zeroPos;
    rArm_stJntPos = rArm_zeroPos;
    //笛卡尔空间
    std::vector<double> lArm_stDescPos, rArm_stDescPos;
    lArm_stDescPos = l_descZeroPos;
    rArm_stDescPos = r_descZeroPos;

    ctlStMsg_v6.desPos_l_Descartes = lArm_stDescPos;
    ctlStMsg_v6.desPos_r_Descartes = rArm_stDescPos;
    ctlStMsg_v6.desPos_l_JointSpace = lArm_stJntPos;
    ctlStMsg_v6.desPos_r_JointSpace = rArm_stJntPos;

    ctlStMsg_v6.desTor_l_Descartes.assign(6, 0.0);
    ctlStMsg_v6.desTor_r_Descartes.assign(6, 0.0);
    ctlStMsg_v6.desTor_l_JointSpace.assign(LARMDOF, 0.0);
    ctlStMsg_v6.desTor_r_JointSpace.assign(RARMDOF, 0.0);

    //关节目标位置1
    //关节空间的目标位置
    std::vector<double> lArm_desJntPos_1, rArm_desJntPos_1;
    lArm_desJntPos_1 = {
        0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5
    };
    rArm_desJntPos_1 = {
        0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5
    };
    //笛卡尔空间目标位置
    std::vector<double> lArm_desDescPos_1, rArm_desDescPos_1;
    lArm_desDescPos_1 = {
        0.35, 0.304998, -0.30, 0.0, -1.5707, 0.0  
        //0.00083588, 0.25449593, -0.553, 0.0, 0.0, 0.0
    };
    rArm_desDescPos_1 = {
        0.35, -0.304998, -0.3, 0.0, -1.5707, 0.0
        //0.00000955, -0.25449594, -0.553, 0.00001102, -0.00002414, -0.00009633
    };

    ctlDesMsgWrap desPoint_1;
    desPoint_1.desPos_l_Descartes = lArm_desDescPos_1;
    desPoint_1.desPos_l_JointSpace = lArm_desJntPos_1;
    desPoint_1.desPos_r_Descartes = rArm_desDescPos_1;
    desPoint_1.desPos_r_JointSpace = rArm_desJntPos_1;

    desPoint_1.desTor_l_Descartes.assign(6, 0.0);
    desPoint_1.desTor_l_JointSpace.assign(7, 0.0);
    desPoint_1.desTor_r_Descartes.assign(6, 0.0);
    desPoint_1.desTor_r_JointSpace.assign(7, 0.0);

    ctlDesPoints_v6.push_back(desPoint_1);

    //关节目标位置2
    //关节空间的目标位置
    std::vector<double> lArm_desJntPos_2, rArm_desJntPos_2;
    lArm_desJntPos_2 = {
        0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5
    };
    rArm_desJntPos_2 = {
        0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5
    };
    //笛卡尔空间目标位置
    std::vector<double> lArm_desDescPos_2, rArm_desDescPos_2;
    lArm_desDescPos_2 = {
        0.30, 0.404998, -0.30, 0.0, -1.5707, 0.0
        //0.00083588, 0.25449593, -0.553,  0.0, 0.0, 0.0
    };
    rArm_desDescPos_2 = {
        0.30, -0.404998, -0.30, 0.0, -1.5707, 0.0
        //0.00000955, -0.25449594, -0.553, 0.00001102, -0.00002414, -0.00009633
    };

    ctlDesMsgWrap desPoint_2;
    desPoint_2.desPos_l_Descartes = lArm_desDescPos_2;
    desPoint_2.desPos_l_JointSpace = lArm_desJntPos_2;
    desPoint_2.desPos_r_Descartes = rArm_desDescPos_2;
    desPoint_2.desPos_r_JointSpace = rArm_desJntPos_2;

    desPoint_2.desTor_l_Descartes.assign(6, 0.0);
    desPoint_2.desTor_l_JointSpace.assign(7, 0.0);
    desPoint_2.desTor_r_Descartes.assign(6, 0.0);
    desPoint_2.desTor_r_JointSpace.assign(7, 0.0);

    ctlDesPoints_v6.push_back(desPoint_2);

    //关节目标位置3
    //关节空间的目标位置
    std::vector<double> lArm_desJntPos_3, rArm_desJntPos_3;
    lArm_desJntPos_3 = {
        0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5
    };
    rArm_desJntPos_3 = {
        0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5
    };

    //笛卡尔空间目标位置
    std::vector<double> lArm_desDescPos_3, rArm_desDescPos_3;
    lArm_desDescPos_3 = {
        0.30, 0.454998, -0.25, 0.0, -1.5707, 0.0 
        //0.000, 0.25, -0.553, 0.000, 0.0, 0.00
    };
    rArm_desDescPos_3 = {
        0.30, -0.454998, -0.25, 0.0, -1.5707, 0.0 
        //0.00000955, -0.25449594, -0.553, 0.00001102, -0.00002414, -0.00009633
    };

    ctlDesMsgWrap desPoint_3;
    desPoint_3.desPos_l_Descartes = lArm_desDescPos_3;
    desPoint_3.desPos_l_JointSpace = lArm_desJntPos_3;
    desPoint_3.desPos_r_Descartes = rArm_desDescPos_3;
    desPoint_3.desPos_r_JointSpace = rArm_desJntPos_3;

    desPoint_3.desTor_l_Descartes.assign(6, 0.0);
    desPoint_3.desTor_l_JointSpace.assign(7, 0.0);
    desPoint_3.desTor_r_Descartes.assign(6, 0.0);
    desPoint_3.desTor_r_JointSpace.assign(7, 0.0);

    ctlDesPoints_v6.push_back(desPoint_3);

    //角度输出模式是否为静止的
    std::vector<int> lArm_ifJntStab, rArm_ifJntStab;
    lArm_ifJntStab = { 1, 1, 1, 1, 1, 1, 1 };
    rArm_ifJntStab = { 0, 0, 0, 0, 0, 0, 0 };
    //lArm_ifJntStab = { 0, 0, 0, 0, 0, 0, 0 };
    //rArm_ifJntStab = { 1, 1, 1, 1, 1, 1, 1 };

    //单个点运动到达次数
    all_singleJntTempTimes_v6 = 7;
    singleJntTempTimes_v6 = 0; 

    //针对所有目标点，分别生成到达轨迹
    //---------------运动轨迹创建---------------
    double m_trajTime = 2.48;      //轨迹时间
    double m_accRate = 1.4;       //加速占比
    int st_len = 10;         // 在初始阶段停留时间
    int des_len = 400;        // 在目标位置停留时间
    
    // 为每个目标点生成往返轨迹
    for (int point_idx = 0; point_idx < ctlDesPoints_v6.size(); point_idx++) {
        ctlDesMsgWrap currentPoint = ctlDesPoints_v6[point_idx];
        std::vector<ctlDesMsgWrap> ctlDesTraj_v6_temp;
        

        // 创建前进轨迹
        std::vector<ctlDesMsgWrap> ctlDesTraj_f;

        std::vector<std::vector<double>> lArm_DescPosTraj;
        std::vector<std::vector<double>> rArm_DescPosTraj;

        std::vector<std::vector<double>> lArm_JntPosTraj;
        std::vector<std::vector<double>> rArm_JntPosTraj;

        lArm_DescPosTraj.resize(6);
        rArm_DescPosTraj.resize(6);

        lArm_JntPosTraj.resize(LARMDOF);
        rArm_JntPosTraj.resize(RARMDOF);

        for(int i = 0; i < 6; i++){

            TrajResult temp_trajRes_l = lArm_DescRuniner_v6[i].dPos_queue_stabTime_dot((ctlDesPoints_v6[point_idx].desPos_l_Descartes[i] - lArm_stDescPos[i]), m_trajTime, m_accRate);
            TrajResult temp_trajRes_r = rArm_DescRuniner_v6[i].dPos_queue_stabTime_dot((ctlDesPoints_v6[point_idx].desPos_r_Descartes[i] - rArm_stDescPos[i]), m_trajTime, m_accRate);

            for(int j = 0; j < temp_trajRes_l.positionIncrements.size(); j++){
                lArm_DescPosTraj[i].push_back(temp_trajRes_l.positionIncrements[j] + lArm_stDescPos[i]);
            }
            for(int j = 0; j < temp_trajRes_r.positionIncrements.size(); j++){
                rArm_DescPosTraj[i].push_back(temp_trajRes_r.positionIncrements[j] + rArm_stDescPos[i]);
            }
        }

        for(int i = 0; i < LARMDOF; i++){
            TrajResult temp_trajRes_l = lArm_JntRuniner_v6[i].dPos_queue_stabTime_dot((ctlDesPoints_v6[point_idx].desPos_l_JointSpace[i] - lArm_stJntPos[i]), m_trajTime, m_accRate);
            TrajResult temp_trajRes_r = rArm_JntRuniner_v6[i].dPos_queue_stabTime_dot((ctlDesPoints_v6[point_idx].desPos_r_JointSpace[i] - rArm_stJntPos[i]), m_trajTime, m_accRate);

            for(int j = 0; j < temp_trajRes_l.positionIncrements.size(); j++){
                lArm_JntPosTraj[i].push_back(temp_trajRes_l.positionIncrements[j] + lArm_stJntPos[i]);
            }
            for(int j = 0; j < temp_trajRes_r.positionIncrements.size(); j++){
                rArm_JntPosTraj[i].push_back(temp_trajRes_r.positionIncrements[j] + rArm_stJntPos[i]);
            }
        }

        for(int i = 0; i < lArm_DescPosTraj[0].size(); i++){
            ctlDesMsgWrap temp_ctlDesMsg;

            temp_ctlDesMsg.desPos_l_Descartes.assign(6, 0.0);
            temp_ctlDesMsg.desPos_r_Descartes.assign(6, 0.0);
            temp_ctlDesMsg.desTor_l_Descartes.assign(6, 0.0);
            temp_ctlDesMsg.desTor_r_Descartes.assign(6, 0.0);

            temp_ctlDesMsg.desPos_l_JointSpace.assign(LARMDOF, 0.0);
            temp_ctlDesMsg.desPos_r_JointSpace.assign(RARMDOF, 0.0);
            temp_ctlDesMsg.desTor_l_JointSpace.assign(LARMDOF, 0.0);
            temp_ctlDesMsg.desTor_r_JointSpace.assign(RARMDOF, 0.0);

            for(int j = 0; j < 6; j++){
                temp_ctlDesMsg.desPos_l_Descartes[j] = lArm_DescPosTraj[j][i];
                temp_ctlDesMsg.desPos_r_Descartes[j] = rArm_DescPosTraj[j][i];    
            }

            for(int j = 0; j < LARMDOF; j++){
                temp_ctlDesMsg.desPos_l_JointSpace[j] = lArm_JntPosTraj[j][i];
                temp_ctlDesMsg.desPos_r_JointSpace[j] = rArm_JntPosTraj[j][i];
            }

            ctlDesTraj_f.push_back(temp_ctlDesMsg);
        }

        // 创建返回轨迹（前进轨迹的反转）
        std::vector<ctlDesMsgWrap> ctlDesTraj_b(ctlDesTraj_f.rbegin(), ctlDesTraj_f.rend());

        // 为当前目标点生成往返轨迹段
        // 起始点停留
        for(int i = 0; i < st_len; i++){
            ctlDesTraj_v6_temp.push_back(ctlStMsg_v6);
        }
        // 前进到目标点
        for(int i = 0; i < ctlDesTraj_f.size(); i++){
            ctlDesTraj_v6_temp.push_back(ctlDesTraj_f[i]);
        }
        // 目标点停留
        for(int i = 0; i < des_len; i++){
            ctlDesTraj_v6_temp.push_back(currentPoint);
        }
        // 返回到起始点
        for(int i = 0; i < ctlDesTraj_b.size(); i++){
            ctlDesTraj_v6_temp.push_back(ctlDesTraj_b[i]);
        }
        // 返回后起始点停留
        for(int i = 0; i < st_len; i++){
            ctlDesTraj_v6_temp.push_back(ctlStMsg_v6);
        }
        ctlDesTraj_v6.push_back(ctlDesTraj_v6_temp);
    }

    // 轨迹总长度
    all_len = ctlDesTraj_v6.size();
    for(int i = 0; i < ctlDesTraj_v6.size(); i++){
        all_len_v6.push_back(ctlDesTraj_v6[i].size());
    }

    //控制值初始化
    double trajTime = 1.4;
    double accRate = 1.8;
    runinStabMsgWrap stabMsg;
    stabMsg.ifUseStab = true;
    stabMsg.start_stabTimes = 50;
    stabMsg.des_stabTimes = 50;

    //关节限位控制模式
    mdataSend.armTrajMsg.JntStabMod_l.ifUseJntLim = 0;
    mdataSend.armTrajMsg.JntStabMod_l.limJntNum = 1;
    mdataSend.armTrajMsg.JntStabMod_l.limJntID = {3};
    mdataSend.armTrajMsg.JntStabMod_l.limJntAng = {0.1};

    mdataSend.armTrajMsg.JntStabMod_r.ifUseJntLim = 0;
    mdataSend.armTrajMsg.JntStabMod_r.limJntNum = 1;
    mdataSend.armTrajMsg.JntStabMod_r.limJntID = {3};
    mdataSend.armTrajMsg.JntStabMod_r.limJntAng = {-0.1};

    //运动控制模式
    mdataSend.armTrajMsg.ctlTransMod = 4;
    mdataSend.armTrajMsg.TorFeedforwardCtl = 0;

    //控制量初始化
    mdataSend.armTrajMsg.ctlDesMsg.lArm_rigidity_mod.assign(7, 1);
    mdataSend.armTrajMsg.ctlDesMsg.rArm_rigidity_mod.assign(7, 1);
    mdataSend.armTrajMsg.ctlDesMsg.lArm_JntKp = lArm_defaultKp_low;
    mdataSend.armTrajMsg.ctlDesMsg.lArm_JntKd = lArm_defaultKd_low;
    mdataSend.armTrajMsg.ctlDesMsg.rArm_JntKp = rArm_defaultKp_low;
    mdataSend.armTrajMsg.ctlDesMsg.rArm_JntKd = rArm_defaultKd_low;

    //单点控制初始化
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.ifUseTemp = 0;         //是否使用循环输出

    double dotTrajTime = trajTime;                                               //轨迹时间2.0s
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.JntTrajTime_L.assign(LARMDOF, dotTrajTime);
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.JntTrajTime_R.assign(RARMDOF, dotTrajTime);
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.JntAccRate_L.assign(LARMDOF, accRate);
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.JntAccRate_R.assign(RARMDOF, accRate);

    //单点位静止消息控制
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.ifReDefineStabJnt = false;
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.lArm_stabMsg.assign(LARMDOF, stabMsg);
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.rArm_stabMsg.assign(RARMDOF, stabMsg);

    //控制量赋值
    mdataSend.armTrajMsg.ctlDesMsg.ifInitUseRet = 0;        //是否使用反馈值

    for(int i = 0; i < LARMDOF; i++){
        mdataSend.armTrajMsg.ctlDesMsg.lArm_rigidity_mod[i] = m_rigidityCtlMod;
        mdataSend.armTrajMsg.ctlDesMsg.rArm_rigidity_mod[i] = m_rigidityCtlMod;
    }

    //针对底盘控制进行赋值
    mdataSend.baseCtlMsg.chassis_cmd.linear_x = 0.0;
    mdataSend.baseCtlMsg.chassis_cmd.linear_y = 0.0;
    mdataSend.baseCtlMsg.chassis_cmd.angular_z = 0.0;

    mdataSend.baseCtlMsg.lifter_cmd.control_mod = 0;        //默认处于遥操作模式
    mdataSend.baseCtlMsg.lifter_cmd.move_cmd = 0;
    mdataSend.baseCtlMsg.lifter_cmd.target_position = 1;
    mdataSend.baseCtlMsg.lifter_cmd.save_preset = false; 

    mdataSend.l_ifJntStab = lArm_ifJntStab;
    mdataSend.r_ifJntStab = rArm_ifJntStab;

    //初始化目标位置
    if(mdataSend.armTrajMsg.ctlTransMod == 1 || mdataSend.armTrajMsg.ctlTransMod == 3){
        mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg = ctlStMsg_v6.desPos_l_JointSpace;
        mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg = ctlStMsg_v6.desPos_r_JointSpace;
    }else{
        mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg = ctlStMsg_v6.desPos_l_Descartes;
        mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg = ctlStMsg_v6.desPos_r_Descartes;
    }

    mdataSend.armTrajMsg.ctlDesMsg.lArm_desTorMsg.assign(LARMDOF, 0.0);
    mdataSend.armTrajMsg.ctlDesMsg.rArm_desTorMsg.assign(RARMDOF, 0.0);
    mdataSend.armTrajMsg.ctlDesMsg.lArm_MDP_desJntVel.assign(LARMDOF, 0.0);
    mdataSend.armTrajMsg.ctlDesMsg.rArm_MDP_desJntVel.assign(RARMDOF, 0.0);
}

DotOutpWrap allDemoTraj::TrajOutp_v6(){
    //输出函数

    if(mdataSend.armTrajMsg.ctlTransMod == 1 || mdataSend.armTrajMsg.ctlTransMod == 3){
        mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg = ctlDesTraj_v6[point_choose_v6][temp_times_v6].desPos_l_JointSpace;
        mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg = ctlDesTraj_v6[point_choose_v6][temp_times_v6].desPos_r_JointSpace;
    }else{
        mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg = ctlDesTraj_v6[point_choose_v6][temp_times_v6].desPos_l_Descartes;
        mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg = ctlDesTraj_v6[point_choose_v6][temp_times_v6].desPos_r_Descartes;
    }
    temp_times_v6 += 1;
    
    if(temp_times_v6 >= all_len_v6[dotID_v6]){

        temp_times_v6 = 0;
        singleJntTempTimes_v6 += 1;

        if(singleJntTempTimes_v6 > all_singleJntTempTimes_v6){
            singleJntTempTimes_v6 = 0;
            dotID_v6 += 1;
            if(dotID_v6 >= 3){
                dotID_v6 = 0;
            }
        }
    }

    return mdataSend;
}


void allDemoTraj::init_v7()
{
    //连续轨迹计算函数

    //关节起始位置
    //关节空间
    std::vector<double> lArm_stJntPos, rArm_stJntPos;
    lArm_stJntPos = lArm_zeroPos;
    rArm_stJntPos = rArm_zeroPos;
    //笛卡尔空间
    std::vector<double> lArm_stDescPos, rArm_stDescPos;
    lArm_stDescPos = l_descZeroPos;
    rArm_stDescPos = r_descZeroPos;

    ctlStMsg_v7.desPos_l_Descartes = lArm_stDescPos;
    ctlStMsg_v7.desPos_r_Descartes = rArm_stDescPos;
    ctlStMsg_v7.desPos_l_JointSpace = lArm_stJntPos;
    ctlStMsg_v7.desPos_r_JointSpace = rArm_stJntPos;

    ctlStMsg_v7.desTor_l_Descartes.assign(6, 0.0);
    ctlStMsg_v7.desTor_r_Descartes.assign(6, 0.0);
    ctlStMsg_v7.desTor_l_JointSpace.assign(LARMDOF, 0.0);
    ctlStMsg_v7.desTor_r_JointSpace.assign(RARMDOF, 0.0);

    //关节目标位置
    //关节空间的目标位置
    std::vector<double> lArm_desJntPos, rArm_desJntPos;
    lArm_desJntPos = {
    };
    rArm_desJntPos = {
    };
    //笛卡尔空间目标位置
    std::vector<double> lArm_desDescPos, rArm_desDescPos;
    lArm_desDescPos = {
        0.00000083, 0.18499646, -0.553, 0.0, 0.0, 0.0 
    };
    rArm_desDescPos = {
        0.00000837, -0.18499573, -0.553, 0.0, 0.0, 0.0
    };

    ctlDesMsg_v7.desPos_l_Descartes = lArm_desDescPos;
    ctlDesMsg_v7.desPos_r_Descartes = rArm_desDescPos;
    ctlDesMsg_v7.desPos_l_JointSpace = lArm_desJntPos;
    ctlDesMsg_v7.desPos_r_JointSpace = rArm_desJntPos;

    ctlDesMsg_v7.desTor_l_Descartes.assign(6, 0.0);
    ctlDesMsg_v7.desTor_r_Descartes.assign(6, 0.0);
    ctlDesMsg_v7.desTor_l_JointSpace.assign(LARMDOF, 0.0);
    ctlDesMsg_v7.desTor_r_JointSpace.assign(RARMDOF, 0.0);

    //角度输出模式是否为静止的
    std::vector<int> lArm_ifJntStab, rArm_ifJntStab;
    lArm_ifJntStab = { 
        0, 0, 0, 0, 0, 0, 0 
    };
    rArm_ifJntStab = { 
        0, 0, 0, 0, 0, 0, 0  
    };

    //控制值初始化
    //-----数据传输与插补模式-----

    //控制值初始化
    double trajTime = 1.4;
    double accRate = 1.8;
    runinStabMsgWrap stabMsg;
    stabMsg.ifUseStab = true;
    stabMsg.start_stabTimes = 50;
    stabMsg.des_stabTimes = 50;

    //关节限位控制模式
    mdataSend.armTrajMsg.JntStabMod_l.ifUseJntLim = 0;
    mdataSend.armTrajMsg.JntStabMod_l.limJntNum = 1;
    mdataSend.armTrajMsg.JntStabMod_l.limJntID = {3};
    mdataSend.armTrajMsg.JntStabMod_l.limJntAng = {0.1};

    mdataSend.armTrajMsg.JntStabMod_r.ifUseJntLim = 0;
    mdataSend.armTrajMsg.JntStabMod_r.limJntNum = 1;
    mdataSend.armTrajMsg.JntStabMod_r.limJntID = {3};
    mdataSend.armTrajMsg.JntStabMod_r.limJntAng = {-0.1};

    //运动控制模式
    mdataSend.armTrajMsg.ctlTransMod = 4;
    mdataSend.armTrajMsg.TorFeedforwardCtl = 0;

    //控制量初始化
    mdataSend.armTrajMsg.ctlDesMsg.lArm_rigidity_mod.assign(7, 1);
    mdataSend.armTrajMsg.ctlDesMsg.rArm_rigidity_mod.assign(7, 1);
    mdataSend.armTrajMsg.ctlDesMsg.lArm_JntKp = lArm_defaultKp_low;
    mdataSend.armTrajMsg.ctlDesMsg.lArm_JntKd = lArm_defaultKd_low;
    mdataSend.armTrajMsg.ctlDesMsg.rArm_JntKp = rArm_defaultKp_low;
    mdataSend.armTrajMsg.ctlDesMsg.rArm_JntKd = rArm_defaultKd_low;

    //单点控制初始化
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.ifUseTemp = 0;         //是否使用循环输出

    double dotTrajTime = trajTime;                                               //轨迹时间2.0s
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.JntTrajTime_L.assign(LARMDOF, dotTrajTime);
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.JntTrajTime_R.assign(RARMDOF, dotTrajTime);
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.JntAccRate_L.assign(LARMDOF, accRate);
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.JntAccRate_R.assign(RARMDOF, accRate);

    //单点位静止消息控制
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.ifReDefineStabJnt = false;
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.lArm_stabMsg.assign(LARMDOF, stabMsg);
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.rArm_stabMsg.assign(RARMDOF, stabMsg);

    //控制量赋值
    mdataSend.armTrajMsg.ctlDesMsg.ifInitUseRet = 0;        //是否使用反馈值

    for(int i = 0; i < LARMDOF; i++){
        mdataSend.armTrajMsg.ctlDesMsg.lArm_rigidity_mod[i] = m_rigidityCtlMod;
        mdataSend.armTrajMsg.ctlDesMsg.rArm_rigidity_mod[i] = m_rigidityCtlMod;
    }

    //针对底盘控制进行赋值
    mdataSend.baseCtlMsg.chassis_cmd.linear_x = 0.0;
    mdataSend.baseCtlMsg.chassis_cmd.linear_y = 0.0;
    mdataSend.baseCtlMsg.chassis_cmd.angular_z = 0.0;

    mdataSend.baseCtlMsg.lifter_cmd.control_mod = 0;        //默认处于遥操作模式
    mdataSend.baseCtlMsg.lifter_cmd.move_cmd = 0;
    mdataSend.baseCtlMsg.lifter_cmd.target_position = 1;
    mdataSend.baseCtlMsg.lifter_cmd.save_preset = false; 

    mdataSend.l_ifJntStab = lArm_ifJntStab;
    mdataSend.r_ifJntStab = rArm_ifJntStab;

    //初始化目标位置
    if(mdataSend.armTrajMsg.ctlTransMod == 1 || mdataSend.armTrajMsg.ctlTransMod == 3){
        mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg = ctlStMsg_v7.desPos_l_JointSpace;
        mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg = ctlStMsg_v7.desPos_r_JointSpace;
    }else{
        mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg = ctlStMsg_v7.desPos_l_Descartes;
        mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg = ctlStMsg_v7.desPos_r_Descartes;
    }

    mdataSend.armTrajMsg.ctlDesMsg.lArm_desTorMsg.assign(LARMDOF, 0.0);
    mdataSend.armTrajMsg.ctlDesMsg.rArm_desTorMsg.assign(RARMDOF, 0.0);
    mdataSend.armTrajMsg.ctlDesMsg.lArm_MDP_desJntVel.assign(LARMDOF, 0.0);
    mdataSend.armTrajMsg.ctlDesMsg.rArm_MDP_desJntVel.assign(RARMDOF, 0.0);
    
}

DotOutpWrap allDemoTraj::TrajOutp_v7()
{
    //控制信息输出
    if(mdataSend.armTrajMsg.ctlTransMod == 1 || mdataSend.armTrajMsg.ctlTransMod == 3){
        mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg = ctlDesMsg_v7.desPos_l_JointSpace;
        mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg = ctlDesMsg_v7.desPos_r_JointSpace;
    }else{
        mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg = ctlDesMsg_v7.desPos_l_Descartes;
        mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg = ctlDesMsg_v7.desPos_r_Descartes;
    }

    return mdataSend;
}

//----------轨迹8----------
//关节空间连续轨迹发送测试

void allDemoTraj::init_v8()
{
    //连续轨迹计算函数

    //插补器初始化：
    lArm_JntRuniner_v8.resize(LARMDOF);
    rArm_JntRuniner_v8.resize(RARMDOF);

    for(int i = 0; i < LARMDOF; i++){
        lArm_JntRuniner_v8[i].Init();
        rArm_JntRuniner_v8[i].Init();
    }

    //---------------始末位置创建---------------
    //关节起始位置
    //关节空间
    std::vector<double> lArm_stJntPos, rArm_stJntPos;
    lArm_stJntPos = lArm_zeroPos;
    rArm_stJntPos = rArm_zeroPos;
    //笛卡尔空间
    std::vector<double> lArm_stDescPos, rArm_stDescPos;
    lArm_stDescPos = l_descZeroPos;
    rArm_stDescPos = r_descZeroPos;

    ctlStMsg_v8.desPos_l_Descartes = lArm_stDescPos;
    ctlStMsg_v8.desPos_r_Descartes = rArm_stDescPos;
    ctlStMsg_v8.desPos_l_JointSpace = lArm_stJntPos;
    ctlStMsg_v8.desPos_r_JointSpace = rArm_stJntPos;

    ctlStMsg_v8.desTor_l_Descartes.assign(6, 0.0);
    ctlStMsg_v8.desTor_r_Descartes.assign(6, 0.0);
    ctlStMsg_v8.desTor_l_JointSpace.assign(LARMDOF, 0.0);
    ctlStMsg_v8.desTor_r_JointSpace.assign(RARMDOF, 0.0);

    //关节目标位置
    //关节空间的目标位置 -- 多关节同时运动，测试关节空间连续轨迹
    std::vector<double> lArm_desJntPos, rArm_desJntPos;
    lArm_desJntPos = {
        0.8, 0.6, 0.4, -1.2, 0.3, 0.3, 0.3
    };
    rArm_desJntPos = {
        0.8, 0.6, 0.4, -1.2, 0.3, 0.3, 0.3
    };
    //笛卡尔空间目标位置（备用）
    std::vector<double> lArm_desDescPos, rArm_desDescPos;
    lArm_desDescPos = {
        0.3, 0.144998, -0.16, -1.7, -0.8, 0.0
    };
    rArm_desDescPos = {
        0.3, -0.144998, -0.16, 1.7, -0.8, 0.0
    };

    ctlDesMsg_v8.desPos_l_Descartes = lArm_desDescPos;
    ctlDesMsg_v8.desPos_r_Descartes = rArm_desDescPos;
    ctlDesMsg_v8.desPos_l_JointSpace = lArm_desJntPos;
    ctlDesMsg_v8.desPos_r_JointSpace = rArm_desJntPos;

    ctlDesMsg_v8.desTor_l_Descartes.assign(6, 0.0);
    ctlDesMsg_v8.desTor_r_Descartes.assign(6, 0.0);
    ctlDesMsg_v8.desTor_l_JointSpace.assign(LARMDOF, 0.0);
    ctlDesMsg_v8.desTor_r_JointSpace.assign(RARMDOF, 0.0);

    //---------------运动轨迹创建---------------

    trajLen_v8 = 200;
    temp_times_v8 = 0;

    double m_trajTime = 0.32;
    double m_accRate = 1.2;
    int st_len = 50;    //在初始阶段停留时间
    int des_len = 50;   //在目标位置停留时间

    //--------------轨迹计数器对象创建---------------
    counter_1_v8 = 0;
    counter_2_v8 = 0;
    all_count_v8 = 3;
    ifReachDesPos_v8 = false;

    //创建轨迹
    std::vector<ctlDesMsgWrap> ctlDesTraj_f;

    std::vector<std::vector<double>> lArm_JntPosTraj;
    std::vector<std::vector<double>> rArm_JntPosTraj;

    lArm_JntPosTraj.resize(LARMDOF);
    rArm_JntPosTraj.resize(RARMDOF);

    //仅使用关节空间插补 -- 测试关节空间连续轨迹发送
    for(int i = 0; i < LARMDOF; i++){
        TrajResult temp_trajRes_l = lArm_JntRuniner_v8[i].dPos_queue_stabTime_dot((lArm_desJntPos[i] - lArm_stJntPos[i]), m_trajTime, m_accRate);
        TrajResult temp_trajRes_r = rArm_JntRuniner_v8[i].dPos_queue_stabTime_dot((rArm_desJntPos[i] - rArm_stJntPos[i]), m_trajTime, m_accRate);

        for(int j = 0; j < temp_trajRes_l.positionIncrements.size(); j++){
            lArm_JntPosTraj[i].push_back(temp_trajRes_l.positionIncrements[j] + lArm_stJntPos[i]);
        }
        for(int j = 0; j < temp_trajRes_r.positionIncrements.size(); j++){
            rArm_JntPosTraj[i].push_back(temp_trajRes_r.positionIncrements[j] + rArm_stJntPos[i]);
        }
    }

    for(int i = 0; i < lArm_JntPosTraj[0].size(); i++){
        ctlDesMsgWrap temp_ctlDesMsg;

        temp_ctlDesMsg.desPos_l_Descartes.assign(6, 0.0);
        temp_ctlDesMsg.desPos_r_Descartes.assign(6, 0.0);
        temp_ctlDesMsg.desTor_l_Descartes.assign(6, 0.0);
        temp_ctlDesMsg.desTor_r_Descartes.assign(6, 0.0);

        temp_ctlDesMsg.desPos_l_JointSpace.assign(LARMDOF, 0.0);
        temp_ctlDesMsg.desPos_r_JointSpace.assign(RARMDOF, 0.0);
        temp_ctlDesMsg.desTor_l_JointSpace.assign(LARMDOF, 0.0);
        temp_ctlDesMsg.desTor_r_JointSpace.assign(RARMDOF, 0.0);

        //笛卡尔空间位置保持初始位置不变（关节空间轨迹测试时笛卡尔空间为辅助）
        for(int j = 0; j < 6; j++){
            temp_ctlDesMsg.desPos_l_Descartes[j] = lArm_stDescPos[j];
            temp_ctlDesMsg.desPos_r_Descartes[j] = rArm_stDescPos[j];
        }

        for(int j = 0; j < LARMDOF; j++){
            temp_ctlDesMsg.desPos_l_JointSpace[j] = lArm_JntPosTraj[j][i];
            temp_ctlDesMsg.desPos_r_JointSpace[j] = rArm_JntPosTraj[j][i];
        }

        ctlDesTraj_f.push_back(temp_ctlDesMsg);
    }

    std::vector<ctlDesMsgWrap> ctlDesTraj_b(ctlDesTraj_f.rbegin(), ctlDesTraj_f.rend());

    //生成总轨迹
    for(int i = 0; i < st_len; i++){
        ctlDesTraj_v8.push_back(ctlStMsg_v8);
    }
    for(int i = 0; i < ctlDesTraj_f.size(); i++){
        ctlDesTraj_v8.push_back(ctlDesTraj_f[i]);
    }
    for(int i = 0; i < des_len; i++){
        ctlDesTraj_v8.push_back(ctlDesMsg_v8);
    }
    for(int i = 0; i < ctlDesTraj_b.size(); i++){
        ctlDesTraj_v8.push_back(ctlDesTraj_b[i]);
    }
    ctlDesTraj_v8.push_back(ctlStMsg_v8);
    all_len_v8 = ctlDesTraj_v8.size();

    counter_1_v8 = st_len + ctlDesTraj_f.size() + 5;


    //----------------------------------------

    //角度输出模式是否为静止的
    std::vector<int> lArm_ifJntStab, rArm_ifJntStab;
    lArm_ifJntStab = { 0, 0, 0, 0, 0, 0, 0 };
    rArm_ifJntStab = { 0, 0, 0, 0, 0, 0, 0 };

    //关节限位控制模式
    mdataSend.armTrajMsg.JntStabMod_l.ifUseJntLim = 0;
    mdataSend.armTrajMsg.JntStabMod_l.limJntNum = 1;
    mdataSend.armTrajMsg.JntStabMod_l.limJntID = {3};
    mdataSend.armTrajMsg.JntStabMod_l.limJntAng = {0.1};

    mdataSend.armTrajMsg.JntStabMod_r.ifUseJntLim = 0;
    mdataSend.armTrajMsg.JntStabMod_r.limJntNum = 1;
    mdataSend.armTrajMsg.JntStabMod_r.limJntID = {3};
    mdataSend.armTrajMsg.JntStabMod_r.limJntAng = {-0.1};

    //运动控制模式
    mdataSend.armTrajMsg.ctlTransMod = 3;
    mdataSend.armTrajMsg.TorFeedforwardCtl = 0;

    //控制量初始化
    mdataSend.armTrajMsg.ctlDesMsg.lArm_rigidity_mod.assign(7, 1);
    mdataSend.armTrajMsg.ctlDesMsg.rArm_rigidity_mod.assign(7, 1);
    mdataSend.armTrajMsg.ctlDesMsg.lArm_JntKp = lArm_defaultKp_low;
    mdataSend.armTrajMsg.ctlDesMsg.lArm_JntKd = lArm_defaultKd_low;
    mdataSend.armTrajMsg.ctlDesMsg.rArm_JntKp = rArm_defaultKp_low;
    mdataSend.armTrajMsg.ctlDesMsg.rArm_JntKd = rArm_defaultKd_low;

    //单点控制初始化
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.ifUseTemp = 0;         //是否使用循环输出

    double dotTrajTime = 0.7;                                               //轨迹时间
    double accRate = 1.2;                                                   //加速时间占比
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.JntTrajTime_L.assign(LARMDOF, dotTrajTime);
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.JntTrajTime_R.assign(RARMDOF, dotTrajTime);
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.JntAccRate_L.assign(LARMDOF, accRate);
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.JntAccRate_R.assign(RARMDOF, accRate);

    //单点位静止消息控制
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.ifReDefineStabJnt = false;
    runinStabMsgWrap runin_stab_msg;
    runin_stab_msg.ifUseStab = true;
    runin_stab_msg.start_stabTimes = 50;
    runin_stab_msg.des_stabTimes = 50;
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.lArm_stabMsg.assign(LARMDOF, runin_stab_msg);
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.rArm_stabMsg.assign(RARMDOF, runin_stab_msg);

    //控制量赋值
    mdataSend.armTrajMsg.ctlDesMsg.ifInitUseRet = 0;        //是否使用反馈值

    if(mdataSend.armTrajMsg.ctlTransMod == 1 || mdataSend.armTrajMsg.ctlTransMod == 3){
        mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg = lArm_stJntPos;
        mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg = rArm_stJntPos;
    }else{
        mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg = lArm_stDescPos;
        mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg = rArm_stDescPos;
    }
    mdataSend.armTrajMsg.ctlDesMsg.lArm_desTorMsg.assign(LARMDOF, 0.0);
    mdataSend.armTrajMsg.ctlDesMsg.rArm_desTorMsg.assign(RARMDOF, 0.0);
    mdataSend.armTrajMsg.ctlDesMsg.lArm_MDP_desJntVel.assign(LARMDOF, 0.0);
    mdataSend.armTrajMsg.ctlDesMsg.rArm_MDP_desJntVel.assign(RARMDOF, 0.0);

    for(int i = 0; i < LARMDOF; i++){
        mdataSend.armTrajMsg.ctlDesMsg.lArm_rigidity_mod[i] = m_rigidityCtlMod;
        mdataSend.armTrajMsg.ctlDesMsg.rArm_rigidity_mod[i] = m_rigidityCtlMod;
    }

    //针对底盘控制进行赋值
    mdataSend.baseCtlMsg.chassis_cmd.linear_x = 0.0;
    mdataSend.baseCtlMsg.chassis_cmd.linear_y = 0.0;
    mdataSend.baseCtlMsg.chassis_cmd.angular_z = 0.0;

    mdataSend.baseCtlMsg.lifter_cmd.control_mod = 0;        //默认处于遥操作模式
    mdataSend.baseCtlMsg.lifter_cmd.move_cmd = 0;
    mdataSend.baseCtlMsg.lifter_cmd.target_position = 1;
    mdataSend.baseCtlMsg.lifter_cmd.save_preset = false;

    mdataSend.l_ifJntStab = lArm_ifJntStab;
    mdataSend.r_ifJntStab = rArm_ifJntStab;
}

DotOutpWrap allDemoTraj::TrajOutp_v8()
{
    if(counter_2_v8 < all_count_v8){
        if(mdataSend.armTrajMsg.ctlTransMod == 1 || mdataSend.armTrajMsg.ctlTransMod == 3){
            mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg = ctlDesTraj_v8[temp_times_v8].desPos_l_JointSpace;
            mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg = ctlDesTraj_v8[temp_times_v8].desPos_r_JointSpace;
        }else{
            mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg = ctlDesTraj_v8[temp_times_v8].desPos_l_Descartes;
            mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg = ctlDesTraj_v8[temp_times_v8].desPos_r_Descartes;
        }

        temp_times_v8 += 1;
        if(temp_times_v8 >= all_len_v8){
            temp_times_v8 = 0;
        }

    }else{
        //先运动到目标位置
        if(!ifReachDesPos_v8){
            if(mdataSend.armTrajMsg.ctlTransMod == 1 || mdataSend.armTrajMsg.ctlTransMod == 3){
                mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg = ctlDesTraj_v8[temp_times_v8].desPos_l_JointSpace;
                mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg = ctlDesTraj_v8[temp_times_v8].desPos_r_JointSpace;
            }else{
                mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg = ctlDesTraj_v8[temp_times_v8].desPos_l_Descartes;
                mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg = ctlDesTraj_v8[temp_times_v8].desPos_r_Descartes;
            }
            if(temp_times_v8 >= counter_1_v8){
                if(mdataSend.armTrajMsg.ctlTransMod == 1 || mdataSend.armTrajMsg.ctlTransMod == 3){
                    mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg = ctlDesTraj_v8[temp_times_v8].desPos_l_JointSpace;
                    mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg = ctlDesTraj_v8[temp_times_v8].desPos_r_JointSpace;
                }else{
                    mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg = ctlDesMsg_v8.desPos_l_Descartes;
                    mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg = ctlDesMsg_v8.desPos_r_Descartes;
                }
                m_rigidityCtlMod = 1;
                for(int k = 0; k < 7; k++){
                    mdataSend.armTrajMsg.ctlDesMsg.lArm_rigidity_mod[k] = m_rigidityCtlMod;
                    mdataSend.armTrajMsg.ctlDesMsg.rArm_rigidity_mod[k] = m_rigidityCtlMod;
                }
            }else{
                temp_times_v8 += 1;
            }
        }
    }

    return mdataSend;
}

//----------轨迹9----------
//余弦扫频轨迹测试 — 关节空间连续轨迹控制模式
//每个关节独立执行余弦扫频信号，频率线性变化（线性 chirp）
//轨迹参数可在 init_v9() 中按需调整

void allDemoTraj::init_v9()
{
    //---------------可调参数---------------
    // 单次正弦轨迹持续时间
    v9_traj_duration = 10.0;        // 正弦轨迹持续时间 (s)
    v9_sample_dt = TempTime;        // 采样间隔 = 控制周期

    // 左臂各关节参数: amplitude(rad), freq(Hz)
    // center 由 lArm_zeroPos / rArm_zeroPos 动态赋值，保证轨迹起始于零位
    v9_lArm_amplitude = { 0.3,  0.3,  0.2,  0.4,  0.2,  0.2,  0.2};
    v9_lArm_freq      = { 0.5,  0.5,  0.5,  0.5,  0.5,  0.5,  0.5};

    // 右臂各关节参数
    v9_rArm_amplitude = { 0.3,  0.3,  0.2,  0.4,  0.2,  0.2,  0.2};
    v9_rArm_freq      = { 0.5,  0.5,  0.5,  0.5,  0.5,  0.5,  0.5};

    // 循环次数
    all_count_v9 = 20;

    //---------------始末位置创建---------------
    // 关节起始位置
    std::vector<double> lArm_stJntPos, rArm_stJntPos;
    lArm_stJntPos = lArm_zeroPos;
    rArm_stJntPos = rArm_zeroPos;

    // 扫频中心设为关节零位，使轨迹初始位置 = 零位
    v9_lArm_center = lArm_zeroPos;
    v9_rArm_center = rArm_zeroPos;
    // 笛卡尔空间起始位置
    std::vector<double> lArm_stDescPos, rArm_stDescPos;
    lArm_stDescPos = l_descZeroPos;
    rArm_stDescPos = r_descZeroPos;

    ctlStMsg_v9.desPos_l_Descartes = lArm_stDescPos;
    ctlStMsg_v9.desPos_r_Descartes = rArm_stDescPos;
    ctlStMsg_v9.desPos_l_JointSpace = lArm_stJntPos;
    ctlStMsg_v9.desPos_r_JointSpace = rArm_stJntPos;
    ctlStMsg_v9.desTor_l_Descartes.assign(6, 0.0);
    ctlStMsg_v9.desTor_r_Descartes.assign(6, 0.0);
    ctlStMsg_v9.desTor_l_JointSpace.assign(LARMDOF, 0.0);
    ctlStMsg_v9.desTor_r_JointSpace.assign(RARMDOF, 0.0);

    //---------------预计算正弦轨迹序列---------------
    // 正弦公式 (固定频率)，使用 sin 使 t=0 时位置 = center（零位）:
    //   位置: pos(t) = center + amplitude * sin(2π * f * t)

    int N = static_cast<int>(std::round(v9_traj_duration / v9_sample_dt));
    if (N < 1) N = 1;
    trajLen_v9 = N;

    ctlDesTraj_v9.clear();
    ctlDesTraj_v9.reserve(N);

    for (int k = 0; k < N; k++)
    {
        double t = k * v9_sample_dt;                    // 当前时间

        ctlDesMsgWrap temp_msg;
        temp_msg.desPos_l_Descartes.assign(6, 0.0);
        temp_msg.desPos_r_Descartes.assign(6, 0.0);
        temp_msg.desTor_l_Descartes.assign(6, 0.0);
        temp_msg.desTor_r_Descartes.assign(6, 0.0);
        temp_msg.desPos_l_JointSpace.assign(LARMDOF, 0.0);
        temp_msg.desPos_r_JointSpace.assign(RARMDOF, 0.0);
        temp_msg.desTor_l_JointSpace.assign(LARMDOF, 0.0);
        temp_msg.desTor_r_JointSpace.assign(RARMDOF, 0.0);

        // 笛卡尔空间保持零位不变
        for (int j = 0; j < 6; j++) {
            temp_msg.desPos_l_Descartes[j] = lArm_stDescPos[j];
            temp_msg.desPos_r_Descartes[j] = rArm_stDescPos[j];
        }

        // 左臂各关节正弦轨迹
        for (int j = 0; j < LARMDOF; j++) {
            double f    = v9_lArm_freq[j];
            double phi  = 2.0 * M_PI * f * t;
            double pos  = v9_lArm_center[j] + v9_lArm_amplitude[j] * std::sin(phi);
            temp_msg.desPos_l_JointSpace[j] = pos;
        }

        // 右臂各关节正弦轨迹
        for (int j = 0; j < RARMDOF; j++) {
            double f    = v9_rArm_freq[j];
            double phi  = 2.0 * M_PI * f * t;
            double pos  = v9_rArm_center[j] + v9_rArm_amplitude[j] * std::sin(phi);
            temp_msg.desPos_r_JointSpace[j] = pos;
        }

        ctlDesTraj_v9.push_back(temp_msg);
    }

    //---------------状态计数器---------------
    temp_times_v9 = 0;
    counter_2_v9 = 0;

    //---------------关节静止模式---------------
    std::vector<int> lArm_ifJntStab, rArm_ifJntStab;
    lArm_ifJntStab = { 1, 1, 1, 1, 1, 1, 0 };
    rArm_ifJntStab = { 1, 1, 1, 1, 1, 1, 0 };

    //---------------运动控制模式---------------
    mdataSend.armTrajMsg.ctlTransMod = 3;       // 关节轨迹控制（连续模式）
    mdataSend.armTrajMsg.TorFeedforwardCtl = 0; // 无力前馈

    // 关节限位控制
    mdataSend.armTrajMsg.JntStabMod_l.ifUseJntLim = 0;
    mdataSend.armTrajMsg.JntStabMod_l.limJntNum = 1;
    mdataSend.armTrajMsg.JntStabMod_l.limJntID = {3};
    mdataSend.armTrajMsg.JntStabMod_l.limJntAng = {0.1};
    mdataSend.armTrajMsg.JntStabMod_r.ifUseJntLim = 0;
    mdataSend.armTrajMsg.JntStabMod_r.limJntNum = 1;
    mdataSend.armTrajMsg.JntStabMod_r.limJntID = {3};
    mdataSend.armTrajMsg.JntStabMod_r.limJntAng = {-0.1};

    // 刚度控制
    mdataSend.armTrajMsg.ctlDesMsg.lArm_rigidity_mod.assign(7, 1);
    mdataSend.armTrajMsg.ctlDesMsg.rArm_rigidity_mod.assign(7, 1);
    mdataSend.armTrajMsg.ctlDesMsg.lArm_JntKp = lArm_defaultKp_low;
    mdataSend.armTrajMsg.ctlDesMsg.lArm_JntKd = lArm_defaultKd_low;
    mdataSend.armTrajMsg.ctlDesMsg.rArm_JntKp = rArm_defaultKp_low;
    mdataSend.armTrajMsg.ctlDesMsg.rArm_JntKd = rArm_defaultKd_low;

    // 单点控制初始化（连续轨迹模式下 runinMod=1, ifUseTemp 不使用）
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.ifUseTemp = 0;
    double dotTrajTime = 0.7;
    double accRate = 1.2;
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.JntTrajTime_L.assign(LARMDOF, dotTrajTime);
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.JntTrajTime_R.assign(RARMDOF, dotTrajTime);
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.JntAccRate_L.assign(LARMDOF, accRate);
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.JntAccRate_R.assign(RARMDOF, accRate);

    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.ifReDefineStabJnt = false;
    runinStabMsgWrap runin_stab_msg;
    runin_stab_msg.ifUseStab = true;
    runin_stab_msg.start_stabTimes = 50;
    runin_stab_msg.des_stabTimes = 50;
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.lArm_stabMsg.assign(LARMDOF, runin_stab_msg);
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.rArm_stabMsg.assign(RARMDOF, runin_stab_msg);

    // 目标量赋值
    mdataSend.armTrajMsg.ctlDesMsg.ifInitUseRet = 0;
    if (mdataSend.armTrajMsg.ctlTransMod == 1 || mdataSend.armTrajMsg.ctlTransMod == 3) {
        mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg = lArm_stJntPos;
        mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg = rArm_stJntPos;
    } else {
        mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg = lArm_stDescPos;
        mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg = rArm_stDescPos;
    }
    mdataSend.armTrajMsg.ctlDesMsg.lArm_desTorMsg.assign(LARMDOF, 0.0);
    mdataSend.armTrajMsg.ctlDesMsg.rArm_desTorMsg.assign(RARMDOF, 0.0);
    mdataSend.armTrajMsg.ctlDesMsg.lArm_MDP_desJntVel.assign(LARMDOF, 0.0);
    mdataSend.armTrajMsg.ctlDesMsg.rArm_MDP_desJntVel.assign(RARMDOF, 0.0);

    for (int i = 0; i < LARMDOF; i++) {
        mdataSend.armTrajMsg.ctlDesMsg.lArm_rigidity_mod[i] = m_rigidityCtlMod;
        mdataSend.armTrajMsg.ctlDesMsg.rArm_rigidity_mod[i] = m_rigidityCtlMod;
    }

    // 底盘控制
    mdataSend.baseCtlMsg.chassis_cmd.linear_x = 0.0;
    mdataSend.baseCtlMsg.chassis_cmd.linear_y = 0.0;
    mdataSend.baseCtlMsg.chassis_cmd.angular_z = 0.0;
    mdataSend.baseCtlMsg.lifter_cmd.control_mod = 0;
    mdataSend.baseCtlMsg.lifter_cmd.move_cmd = 0;
    mdataSend.baseCtlMsg.lifter_cmd.target_position = 1;
    mdataSend.baseCtlMsg.lifter_cmd.save_preset = false;

    mdataSend.l_ifJntStab = lArm_ifJntStab;
    mdataSend.r_ifJntStab = rArm_ifJntStab;

    std::cout << "[轨迹9] 正弦轨迹初始化完成:" << std::endl;
    std::cout << "  轨迹持续时间: " << v9_traj_duration << " s" << std::endl;
    std::cout << "  轨迹点数: " << trajLen_v9 << std::endl;
    std::cout << "  循环次数: " << all_count_v9 << std::endl;
    std::cout << "  控制模式: transMod=3 (关节轨迹控制, 连续)" << std::endl;
}

DotOutpWrap allDemoTraj::TrajOutp_v9()
{
    // 输出当前轨迹点并推进索引
    if (temp_times_v9 < static_cast<int>(ctlDesTraj_v9.size())) {
        if (mdataSend.armTrajMsg.ctlTransMod == 1 ||
            mdataSend.armTrajMsg.ctlTransMod == 3) {
            mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg =
                ctlDesTraj_v9[temp_times_v9].desPos_l_JointSpace;
            mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg =
                ctlDesTraj_v9[temp_times_v9].desPos_r_JointSpace;
        } else {
            mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg =
                ctlDesTraj_v9[temp_times_v9].desPos_l_Descartes;
            mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg =
                ctlDesTraj_v9[temp_times_v9].desPos_r_Descartes;
        }
    }

    temp_times_v9 += 1;

    // 一轮扫频完成：重置索引并计数
    if (temp_times_v9 >= trajLen_v9) {
        temp_times_v9 = 0;
        counter_2_v9 += 1;

        std::cout << "[轨迹9] 正弦循环 " << counter_2_v9
                  << " / " << all_count_v9 << " 完成" << std::endl;
    }

    // 全部循环结束：停在零位
    if (counter_2_v9 >= all_count_v9) {
        mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg =
            ctlStMsg_v9.desPos_l_JointSpace;
        mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg =
            ctlStMsg_v9.desPos_r_JointSpace;
    }

    return mdataSend;
}

//----------轨迹10----------
//正弦轨迹测试 — MDP传输模式 (ctlTransMod=6)
//与 v9 不同，MDP 模式同时传输关节目标位置和目标速度，
//dual_arm_control_hw 直接使用这些值，不再内部插补

void allDemoTraj::init_v10()
{
    //---------------可调参数---------------
    v10_trajDuration = 10.0;            // 正弦轨迹持续时间 (s)
    v10_sampleDt = TempTime;            // 采样间隔 = 控制周期

    // 左臂各关节参数: amplitude(rad), freq(Hz)
    // center 由 lArm_zeroPos 赋值，使轨迹起始于零位
    v10_lArm_amplitude = { 0.3,  0.3,  0.2,  0.4,  0.2,  0.2,  0.2};
    v10_lArm_freq      = { 0.5,  0.5,  0.5,  0.5,  0.5,  0.5,  0.5};

    // 右臂各关节参数
    v10_rArm_amplitude = { 0.3,  0.3,  0.2,  0.4,  0.2,  0.2,  0.2};
    v10_rArm_freq      = { 0.5,  0.5,  0.5,  0.5,  0.5,  0.5,  0.5};

    // 循环次数
    v10_allCount = 4;

    // 扫频中心 = 关节零位
    v10_lArm_center = lArm_zeroPos;
    v10_rArm_center = rArm_zeroPos;

    //---------------预计算正弦轨迹序列---------------
    // 正弦公式 (固定频率):
    //   pos(t) = center + amplitude * sin(2π * f * t)
    //   vel(t) = amplitude * 2π * f * cos(2π * f * t)

    int N = static_cast<int>(std::round(v10_trajDuration / v10_sampleDt));
    if (N < 1) N = 1;
    v10_trajLen = N;

    v10_lArm_PosTraj.assign(N, std::vector<double>(LARMDOF, 0.0));
    v10_rArm_PosTraj.assign(N, std::vector<double>(RARMDOF, 0.0));
    v10_lArm_VelTraj.assign(N, std::vector<double>(LARMDOF, 0.0));
    v10_rArm_VelTraj.assign(N, std::vector<double>(RARMDOF, 0.0));

    for (int k = 0; k < N; k++)
    {
        double t = k * v10_sampleDt;

        // 左臂各关节
        for (int j = 0; j < LARMDOF; j++) {
            double f   = v10_lArm_freq[j];
            double phi = 2.0 * M_PI * f * t;
            v10_lArm_PosTraj[k][j] = v10_lArm_center[j]
                                     + v10_lArm_amplitude[j] * std::sin(phi);
            v10_lArm_VelTraj[k][j] = v10_lArm_amplitude[j]
                                     * 2.0 * M_PI * f * std::cos(phi);
        }

        // 右臂各关节
        for (int j = 0; j < RARMDOF; j++) {
            double f   = v10_rArm_freq[j];
            double phi = 2.0 * M_PI * f * t;
            v10_rArm_PosTraj[k][j] = v10_rArm_center[j]
                                     + v10_rArm_amplitude[j] * std::sin(phi);
            v10_rArm_VelTraj[k][j] = v10_rArm_amplitude[j]
                                     * 2.0 * M_PI * f * std::cos(phi);
        }
    }

    //---------------状态计数器---------------
    v10_tempTimes = 0;
    v10_counter2 = 0;

    //---------------关节静止模式---------------
    std::vector<int> lArm_ifJntStab, rArm_ifJntStab;
    lArm_ifJntStab = { 0, 1, 1, 1, 1, 1, 1 };
    rArm_ifJntStab = { 1, 1, 1, 1, 1, 1, 1 };

    //---------------运动控制模式---------------
    mdataSend.armTrajMsg.ctlTransMod = 6;       // MDP 模式
    mdataSend.armTrajMsg.TorFeedforwardCtl = 0;

    // 关节限位控制
    mdataSend.armTrajMsg.JntStabMod_l.ifUseJntLim = 0;
    mdataSend.armTrajMsg.JntStabMod_l.limJntNum = 1;
    mdataSend.armTrajMsg.JntStabMod_l.limJntID = {3};
    mdataSend.armTrajMsg.JntStabMod_l.limJntAng = {0.1};
    mdataSend.armTrajMsg.JntStabMod_r.ifUseJntLim = 0;
    mdataSend.armTrajMsg.JntStabMod_r.limJntNum = 1;
    mdataSend.armTrajMsg.JntStabMod_r.limJntID = {3};
    mdataSend.armTrajMsg.JntStabMod_r.limJntAng = {-0.1};

    // 刚度控制
    mdataSend.armTrajMsg.ctlDesMsg.lArm_rigidity_mod.assign(7, 1);
    mdataSend.armTrajMsg.ctlDesMsg.rArm_rigidity_mod.assign(7, 1);
    mdataSend.armTrajMsg.ctlDesMsg.lArm_JntKp = lArm_defaultKp_low;
    mdataSend.armTrajMsg.ctlDesMsg.lArm_JntKd = lArm_defaultKd_low;
    mdataSend.armTrajMsg.ctlDesMsg.rArm_JntKp = rArm_defaultKp_low;
    mdataSend.armTrajMsg.ctlDesMsg.rArm_JntKd = rArm_defaultKd_low;

    // MDP 模式下不使用点位控制参数
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.ifUseTemp = 0;
    double dotTrajTime = 0.7;
    double accRate = 1.2;
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.JntTrajTime_L.assign(LARMDOF, dotTrajTime);
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.JntTrajTime_R.assign(RARMDOF, dotTrajTime);
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.JntAccRate_L.assign(LARMDOF, accRate);
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.JntAccRate_R.assign(RARMDOF, accRate);
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.ifReDefineStabJnt = false;
    runinStabMsgWrap runin_stab_msg;
    runin_stab_msg.ifUseStab = true;
    runin_stab_msg.start_stabTimes = 50;
    runin_stab_msg.des_stabTimes = 50;
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.lArm_stabMsg.assign(LARMDOF, runin_stab_msg);
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.rArm_stabMsg.assign(RARMDOF, runin_stab_msg);

    // 目标量赋值 — MDP 模式 lArm_desMsg 装关节位置，lArm_MDP_desJntVel 装关节速度
    mdataSend.armTrajMsg.ctlDesMsg.ifInitUseRet = 0;
    mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg = lArm_zeroPos;
    mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg = rArm_zeroPos;
    mdataSend.armTrajMsg.ctlDesMsg.lArm_desTorMsg.assign(LARMDOF, 0.0);
    mdataSend.armTrajMsg.ctlDesMsg.rArm_desTorMsg.assign(RARMDOF, 0.0);
    mdataSend.armTrajMsg.ctlDesMsg.lArm_MDP_desJntVel.assign(LARMDOF, 0.0);
    mdataSend.armTrajMsg.ctlDesMsg.rArm_MDP_desJntVel.assign(RARMDOF, 0.0);

    for (int i = 0; i < LARMDOF; i++) {
        mdataSend.armTrajMsg.ctlDesMsg.lArm_rigidity_mod[i] = m_rigidityCtlMod;
        mdataSend.armTrajMsg.ctlDesMsg.rArm_rigidity_mod[i] = m_rigidityCtlMod;
    }

    // 底盘控制
    mdataSend.baseCtlMsg.chassis_cmd.linear_x = 0.0;
    mdataSend.baseCtlMsg.chassis_cmd.linear_y = 0.0;
    mdataSend.baseCtlMsg.chassis_cmd.angular_z = 0.0;
    mdataSend.baseCtlMsg.lifter_cmd.control_mod = 0;
    mdataSend.baseCtlMsg.lifter_cmd.move_cmd = 0;
    mdataSend.baseCtlMsg.lifter_cmd.target_position = 1;
    mdataSend.baseCtlMsg.lifter_cmd.save_preset = false;

    mdataSend.l_ifJntStab = lArm_ifJntStab;
    mdataSend.r_ifJntStab = rArm_ifJntStab;

    std::cout << "[轨迹10] 正弦 MDP 模式初始化完成:" << std::endl;
    std::cout << "  轨迹持续时间: " << v10_trajDuration << " s" << std::endl;
    std::cout << "  轨迹点数: " << v10_trajLen << std::endl;
    std::cout << "  循环次数: " << v10_allCount << std::endl;
    std::cout << "  控制模式: transMod=6 (MDP, 关节位置+速度透传)" << std::endl;
}

DotOutpWrap allDemoTraj::TrajOutp_v10()
{
    // 输出当前轨迹点的位置和速度
    if (v10_tempTimes < v10_trajLen) {
        mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg =
            v10_lArm_PosTraj[v10_tempTimes];
        mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg =
            v10_rArm_PosTraj[v10_tempTimes];

        // MDP 模式关键：同时下发目标速度
        mdataSend.armTrajMsg.ctlDesMsg.lArm_MDP_desJntVel =
            v10_lArm_VelTraj[v10_tempTimes];
        mdataSend.armTrajMsg.ctlDesMsg.rArm_MDP_desJntVel =
            v10_rArm_VelTraj[v10_tempTimes];
    }

    v10_tempTimes += 1;

    // 一轮扫频完成：重置索引并计数
    if (v10_tempTimes >= v10_trajLen) {
        v10_tempTimes = 0;
        v10_counter2 += 1;

        std::cout << "[轨迹10] 正弦循环 " << v10_counter2
                  << " / " << v10_allCount << " 完成" << std::endl;
    }

    // 全部循环结束：停在零位，速度归零
    if (v10_counter2 >= v10_allCount) {
        mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg = lArm_zeroPos;
        mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg = rArm_zeroPos;
        mdataSend.armTrajMsg.ctlDesMsg.lArm_MDP_desJntVel.assign(LARMDOF, 0.0);
        mdataSend.armTrajMsg.ctlDesMsg.rArm_MDP_desJntVel.assign(RARMDOF, 0.0);
    }

    global_debugPub->debug_msg_publish_1(mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg, mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg);

    return mdataSend;
}

//----------轨迹11----------
//控制模式切换测试轨迹
//模式1: 笛卡尔空间 点位(transMod=2)→轨迹(transMod=4)→点位(transMod=2) 循环
//模式2: 关节空间   点位(transMod=1)→轨迹(transMod=3)→点位(transMod=1) 循环
//
//状态机: phase 0→1→2→0 循环
//  phase 0: 点位模式到达中继点(A或C)
//  phase 1: 连续轨迹模式到达目标点(B或D)
//  phase 2: 点位模式回到初始位置

void allDemoTraj::init_v11()
{
    std::cout << "请选择控制模式切换测试类型:" << std::endl;
    std::cout << "  1 -- 笛卡尔空间混合切换" << std::endl;
    std::cout << "  2 -- 关节空间混合切换" << std::endl;
    std::cin >> v11_mode;
    if (v11_mode < 1 || v11_mode > 2) {
        std::cout << "输入有误，默认使用模式1（笛卡尔空间）" << std::endl;
        v11_mode = 1;
    }

    //---------------可调参数---------------
    v11_allCount = 3;       // 总循环次数
    int trajSteps = 200;    // 轨迹段插值步数
    int stLen     = 50;     // 轨迹起始段停留步数
    int desLen    = 50;     // 轨迹结束段停留步数

    //---------------初始位置---------------
    std::vector<double> lArm_stJntPos = lArm_zeroPos;
    std::vector<double> rArm_stJntPos = rArm_zeroPos;
    std::vector<double> lArm_stDescPos = l_descZeroPos;
    std::vector<double> rArm_stDescPos = r_descZeroPos;

    v11_ctlStMsg.desPos_l_Descartes = lArm_stDescPos;
    v11_ctlStMsg.desPos_r_Descartes = rArm_stDescPos;
    v11_ctlStMsg.desPos_l_JointSpace = lArm_stJntPos;
    v11_ctlStMsg.desPos_r_JointSpace = rArm_stJntPos;
    v11_ctlStMsg.desTor_l_Descartes.assign(6, 0.0);
    v11_ctlStMsg.desTor_r_Descartes.assign(6, 0.0);
    v11_ctlStMsg.desTor_l_JointSpace.assign(LARMDOF, 0.0);
    v11_ctlStMsg.desTor_r_JointSpace.assign(RARMDOF, 0.0);

    //---------------定义中间节点---------------
    // 笛卡尔空间 A, B (6 DOF: x,y,z,roll,pitch,yaw)
    std::vector<double> lArm_A_Desc = {
        0.3, 0.145, -0.16, -1.7, -0.8, 0.0
    };
    std::vector<double> rArm_A_Desc = {
        0.3, -0.145, -0.16, 1.7, -0.8, 0.0
    };
    std::vector<double> lArm_B_Desc = {
        0.35, 0.255, -0.309, 0.0, -1.57, 0.0
    };
    std::vector<double> rArm_B_Desc = {
        0.35, -0.255, -0.309, 0.0, -1.57, 0.0
    };

    // 关节空间 C, D (LARMDOF DOF)
    std::vector<double> lArm_C_Jnt = {
        0.0, 0.0, 0.0, -1.57, 0.0, 0.0, 0.0
    };
    std::vector<double> rArm_C_Jnt = {
        0.0, 0.0, 0.0, -1.57, 0.0, 0.0, 0.0
    };
    std::vector<double> lArm_D_Jnt = {
        -0.5, 0.0, 0.0, 1.07, 0.0, 0.0, 0.0
    };
    std::vector<double> rArm_D_Jnt = {
        -0.5, 0.0, 0.0, -1.07, 0.0, 0.0, 0.0
    };

    // 填充 ctlDesMsgWrap
    v11_pointA.desPos_l_Descartes = lArm_A_Desc;
    v11_pointA.desPos_r_Descartes = rArm_A_Desc;
    v11_pointA.desPos_l_JointSpace.assign(LARMDOF, 0.0);
    v11_pointA.desPos_r_JointSpace.assign(RARMDOF, 0.0);
    v11_pointA.desTor_l_Descartes.assign(6, 0.0);
    v11_pointA.desTor_r_Descartes.assign(6, 0.0);
    v11_pointA.desTor_l_JointSpace.assign(LARMDOF, 0.0);
    v11_pointA.desTor_r_JointSpace.assign(RARMDOF, 0.0);

    v11_pointB.desPos_l_Descartes = lArm_B_Desc;
    v11_pointB.desPos_r_Descartes = rArm_B_Desc;
    v11_pointB.desPos_l_JointSpace.assign(LARMDOF, 0.0);
    v11_pointB.desPos_r_JointSpace.assign(RARMDOF, 0.0);
    v11_pointB.desTor_l_Descartes.assign(6, 0.0);
    v11_pointB.desTor_r_Descartes.assign(6, 0.0);
    v11_pointB.desTor_l_JointSpace.assign(LARMDOF, 0.0);
    v11_pointB.desTor_r_JointSpace.assign(RARMDOF, 0.0);

    v11_pointC.desPos_l_Descartes.assign(6, 0.0);
    v11_pointC.desPos_r_Descartes.assign(6, 0.0);
    v11_pointC.desPos_l_JointSpace = lArm_C_Jnt;
    v11_pointC.desPos_r_JointSpace = rArm_C_Jnt;
    v11_pointC.desTor_l_Descartes.assign(6, 0.0);
    v11_pointC.desTor_r_Descartes.assign(6, 0.0);
    v11_pointC.desTor_l_JointSpace.assign(LARMDOF, 0.0);
    v11_pointC.desTor_r_JointSpace.assign(RARMDOF, 0.0);

    v11_pointD.desPos_l_Descartes.assign(6, 0.0);
    v11_pointD.desPos_r_Descartes.assign(6, 0.0);
    v11_pointD.desPos_l_JointSpace = lArm_D_Jnt;
    v11_pointD.desPos_r_JointSpace = rArm_D_Jnt;
    v11_pointD.desTor_l_Descartes.assign(6, 0.0);
    v11_pointD.desTor_r_Descartes.assign(6, 0.0);
    v11_pointD.desTor_l_JointSpace.assign(LARMDOF, 0.0);
    v11_pointD.desTor_r_JointSpace.assign(RARMDOF, 0.0);

    //---------------预计算轨迹序列 (phase 1: A→B 或 C→D)---------------
    // 结构: [stLen个起点] + [trajSteps个插值点] + [desLen个终点]
    v11_trajLen = stLen + trajSteps + desLen;
    v11_traj.clear();
    v11_traj.resize(v11_trajLen);

    if (v11_mode == 1) {
        // 笛卡尔空间: 线性插值 A→B
        for (int i = 0; i < stLen; i++) {
            v11_traj[i] = v11_pointA;  // 停留段
        }
        for (int i = 0; i < trajSteps; i++) {
            double alpha = static_cast<double>(i + 1) / trajSteps;
            ctlDesMsgWrap pt;
            pt.desPos_l_Descartes.resize(6);
            pt.desPos_r_Descartes.resize(6);
            pt.desPos_l_JointSpace.assign(LARMDOF, 0.0);
            pt.desPos_r_JointSpace.assign(RARMDOF, 0.0);
            pt.desTor_l_Descartes.assign(6, 0.0);
            pt.desTor_r_Descartes.assign(6, 0.0);
            pt.desTor_l_JointSpace.assign(LARMDOF, 0.0);
            pt.desTor_r_JointSpace.assign(RARMDOF, 0.0);
            for (int j = 0; j < 6; j++) {
                pt.desPos_l_Descartes[j] = lArm_A_Desc[j]
                    + alpha * (lArm_B_Desc[j] - lArm_A_Desc[j]);
                pt.desPos_r_Descartes[j] = rArm_A_Desc[j]
                    + alpha * (rArm_B_Desc[j] - rArm_A_Desc[j]);
            }
            v11_traj[stLen + i] = pt;
        }
        for (int i = 0; i < desLen; i++) {
            v11_traj[stLen + trajSteps + i] = v11_pointB;  // 停留段
        }
    } else {
        // 关节空间: 线性插值 C→D
        for (int i = 0; i < stLen; i++) {
            v11_traj[i] = v11_pointC;
        }
        for (int i = 0; i < trajSteps; i++) {
            double alpha = static_cast<double>(i + 1) / trajSteps;
            ctlDesMsgWrap pt;
            pt.desPos_l_Descartes.assign(6, 0.0);
            pt.desPos_r_Descartes.assign(6, 0.0);
            pt.desPos_l_JointSpace.resize(LARMDOF);
            pt.desPos_r_JointSpace.resize(RARMDOF);
            pt.desTor_l_Descartes.assign(6, 0.0);
            pt.desTor_r_Descartes.assign(6, 0.0);
            pt.desTor_l_JointSpace.assign(LARMDOF, 0.0);
            pt.desTor_r_JointSpace.assign(RARMDOF, 0.0);
            for (int j = 0; j < LARMDOF; j++) {
                pt.desPos_l_JointSpace[j] = lArm_C_Jnt[j]
                    + alpha * (lArm_D_Jnt[j] - lArm_C_Jnt[j]);
                pt.desPos_r_JointSpace[j] = rArm_C_Jnt[j]
                    + alpha * (rArm_D_Jnt[j] - rArm_C_Jnt[j]);
            }
            v11_traj[stLen + i] = pt;
        }
        for (int i = 0; i < desLen; i++) {
            v11_traj[stLen + trajSteps + i] = v11_pointD;
        }
    }

    //---------------状态机初始化---------------
    v11_phase = 0;
    v11_counter = 0;
    v11_justEnteredPhase = true;
    v11_trajIdx = 0;

    //---------------关节静止模式---------------
    std::vector<int> lArm_ifJntStab, rArm_ifJntStab;
    lArm_ifJntStab = { 0, 0, 0, 0, 0, 0, 0 };
    rArm_ifJntStab = { 0, 0, 0, 0, 0, 0, 0 };

    //---------------mdataSend 公共部分---------------
    // 关节限位控制
    mdataSend.armTrajMsg.JntStabMod_l.ifUseJntLim = 0;
    mdataSend.armTrajMsg.JntStabMod_l.limJntNum = 1;
    mdataSend.armTrajMsg.JntStabMod_l.limJntID = {3};
    mdataSend.armTrajMsg.JntStabMod_l.limJntAng = {0.1};
    mdataSend.armTrajMsg.JntStabMod_r.ifUseJntLim = 0;
    mdataSend.armTrajMsg.JntStabMod_r.limJntNum = 1;
    mdataSend.armTrajMsg.JntStabMod_r.limJntID = {3};
    mdataSend.armTrajMsg.JntStabMod_r.limJntAng = {-0.1};

    mdataSend.armTrajMsg.TorFeedforwardCtl = 0;

    // 刚度控制
    mdataSend.armTrajMsg.ctlDesMsg.lArm_rigidity_mod.assign(7, 1);
    mdataSend.armTrajMsg.ctlDesMsg.rArm_rigidity_mod.assign(7, 1);
    mdataSend.armTrajMsg.ctlDesMsg.lArm_JntKp = lArm_defaultKp_low;
    mdataSend.armTrajMsg.ctlDesMsg.lArm_JntKd = lArm_defaultKd_low;
    mdataSend.armTrajMsg.ctlDesMsg.rArm_JntKp = rArm_defaultKp_low;
    mdataSend.armTrajMsg.ctlDesMsg.rArm_JntKd = rArm_defaultKd_low;

    // 点位控制参数
    double dotTrajTime = 2.0;
    double accRate = 1.2;
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.ifUseTemp = 0;
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.JntTrajTime_L.assign(LARMDOF, dotTrajTime);
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.JntTrajTime_R.assign(RARMDOF, dotTrajTime);
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.JntAccRate_L.assign(LARMDOF, accRate);
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.JntAccRate_R.assign(RARMDOF, accRate);
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.ifReDefineStabJnt = false;
    runinStabMsgWrap runin_stab_msg;
    runin_stab_msg.ifUseStab = true;
    runin_stab_msg.start_stabTimes = 50;
    runin_stab_msg.des_stabTimes = 50;
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.lArm_stabMsg.assign(LARMDOF, runin_stab_msg);
    mdataSend.armTrajMsg.ctlDesMsg.m_SingleDotCtlMsg.rArm_stabMsg.assign(RARMDOF, runin_stab_msg);

    // 不使用反馈值
    mdataSend.armTrajMsg.ctlDesMsg.ifInitUseRet = 0;

    // 初始目标 = 零位
    mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg = lArm_stJntPos;
    mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg = rArm_stJntPos;
    mdataSend.armTrajMsg.ctlDesMsg.lArm_desTorMsg.assign(LARMDOF, 0.0);
    mdataSend.armTrajMsg.ctlDesMsg.rArm_desTorMsg.assign(RARMDOF, 0.0);
    mdataSend.armTrajMsg.ctlDesMsg.lArm_MDP_desJntVel.assign(LARMDOF, 0.0);
    mdataSend.armTrajMsg.ctlDesMsg.rArm_MDP_desJntVel.assign(RARMDOF, 0.0);

    for (int i = 0; i < LARMDOF; i++) {
        mdataSend.armTrajMsg.ctlDesMsg.lArm_rigidity_mod[i] = m_rigidityCtlMod;
        mdataSend.armTrajMsg.ctlDesMsg.rArm_rigidity_mod[i] = m_rigidityCtlMod;
    }

    // 底盘控制
    mdataSend.baseCtlMsg.chassis_cmd.linear_x = 0.0;
    mdataSend.baseCtlMsg.chassis_cmd.linear_y = 0.0;
    mdataSend.baseCtlMsg.chassis_cmd.angular_z = 0.0;
    mdataSend.baseCtlMsg.lifter_cmd.control_mod = 0;
    mdataSend.baseCtlMsg.lifter_cmd.move_cmd = 0;
    mdataSend.baseCtlMsg.lifter_cmd.target_position = 1;
    mdataSend.baseCtlMsg.lifter_cmd.save_preset = false;

    mdataSend.l_ifJntStab = lArm_ifJntStab;
    mdataSend.r_ifJntStab = rArm_ifJntStab;

    // 初始模式: 根据选择设定
    if (v11_mode == 1) {
        mdataSend.armTrajMsg.ctlTransMod = 2;  // 笛卡尔点位
    } else {
        mdataSend.armTrajMsg.ctlTransMod = 1;  // 关节点位
    }

    std::cout << "[轨迹11] 控制模式切换测试轨迹初始化完成:" << std::endl;
    std::cout << "  测试模式: " << (v11_mode == 1 ? "笛卡尔空间混合切换" : "关节空间混合切换") << std::endl;
    std::cout << "  轨迹点数: " << v11_trajLen << " (停留" << stLen
              << " + 插值" << trajSteps << " + 停留" << desLen << ")" << std::endl;
    std::cout << "  循环次数: " << v11_allCount << std::endl;
}

DotOutpWrap allDemoTraj::TrajOutp_v11()
{
    // ===== 状态机主循环 =====
    // phase 0: 点位模式 → 中继点 (A 或 C)
    // phase 1: 轨迹模式 → 目标点 (B 或 D)
    // phase 2: 点位模式 → 初始位置
    // 全部循环结束后停在初始位置

    if (v11_counter >= v11_allCount) {
        // 全部循环完成: 保持在初始位置
        mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg = v11_ctlStMsg.desPos_l_JointSpace;
        mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg = v11_ctlStMsg.desPos_r_JointSpace;
        return mdataSend;
    }

    bool phaseHandled = false;
    while (!phaseHandled) {
        switch (v11_phase) {

        case 0: {
            // ---- 点位模式 → A(笛卡尔) 或 C(关节) ----
            if (v11_justEnteredPhase) {
                // 首次进入: 切换到点位模式，设置目标
                if (v11_mode == 1) {
                    mdataSend.armTrajMsg.ctlTransMod = 2;  // 笛卡尔点位
                    mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg =
                        v11_pointA.desPos_l_Descartes;
                    mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg =
                        v11_pointA.desPos_r_Descartes;
                } else {
                    mdataSend.armTrajMsg.ctlTransMod = 1;  // 关节点位
                    mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg =
                        v11_pointC.desPos_l_JointSpace;
                    mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg =
                        v11_pointC.desPos_r_JointSpace;
                }
                v11_justEnteredPhase = false;
                phaseHandled = true;
            } else {
                // 已到达目标: 切换到下一阶段
                v11_phase = 1;
                v11_justEnteredPhase = true;
                // 继续循环, 处理 phase 1
            }
            break;
        }

        case 1: {
            // ---- 轨迹模式 A→B(笛卡尔) 或 C→D(关节) ----
            if (v11_justEnteredPhase) {
                // 首次进入: 切换到连续轨迹模式, 重置索引
                if (v11_mode == 1) {
                    mdataSend.armTrajMsg.ctlTransMod = 4;  // 笛卡尔轨迹
                } else {
                    mdataSend.armTrajMsg.ctlTransMod = 3;  // 关节轨迹
                }
                v11_trajIdx = 0;
                v11_justEnteredPhase = false;
                // 继续输出第一个轨迹点
            }

            if (v11_trajIdx < v11_trajLen) {
                // 输出当前轨迹点
                if (v11_mode == 1) {
                    // 笛卡尔模式: 使用 desPos_l_Descartes
                    mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg =
                        v11_traj[v11_trajIdx].desPos_l_Descartes;
                    mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg =
                        v11_traj[v11_trajIdx].desPos_r_Descartes;
                } else {
                    // 关节模式: 使用 desPos_l_JointSpace
                    mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg =
                        v11_traj[v11_trajIdx].desPos_l_JointSpace;
                    mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg =
                        v11_traj[v11_trajIdx].desPos_r_JointSpace;
                }
                v11_trajIdx++;
                phaseHandled = true;
            } else {
                // 轨迹序列输出完毕: 切换到 phase 2
                v11_phase = 2;
                v11_justEnteredPhase = true;
                // 继续循环, 处理 phase 2
            }
            break;
        }

        case 2: {
            // ---- 点位模式 → 初始位置 ----
            if (v11_justEnteredPhase) {
                // 首次进入: 切换到点位模式，目标=初始位置
                if (v11_mode == 1) {
                    mdataSend.armTrajMsg.ctlTransMod = 2;  // 笛卡尔点位
                    mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg =
                        v11_ctlStMsg.desPos_l_Descartes;
                    mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg =
                        v11_ctlStMsg.desPos_r_Descartes;
                } else {
                    mdataSend.armTrajMsg.ctlTransMod = 1;  // 关节点位
                    mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg =
                        v11_ctlStMsg.desPos_l_JointSpace;
                    mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg =
                        v11_ctlStMsg.desPos_r_JointSpace;
                }
                v11_justEnteredPhase = false;
                phaseHandled = true;
            } else {
                // 已回到初始位置: 循环计数+1, 回到 phase 0
                v11_counter++;
                std::cout << "[轨迹11] 循环 " << v11_counter
                          << " / " << v11_allCount << " 完成" << std::endl;

                if (v11_counter >= v11_allCount) {
                    // 全部完成: 停在初始位置
                    mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg =
                        v11_ctlStMsg.desPos_l_JointSpace;
                    mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg =
                        v11_ctlStMsg.desPos_r_JointSpace;
                    phaseHandled = true;
                } else {
                    v11_phase = 0;
                    v11_justEnteredPhase = true;
                    // 继续循环，处理 phase 0
                }
            }
            break;
        }

        } // switch
    } // while

    global_debugPub->debug_msg_publish_1(
        mdataSend.armTrajMsg.ctlDesMsg.lArm_desMsg,
        mdataSend.armTrajMsg.ctlDesMsg.rArm_desMsg);

    return mdataSend;
}

//===============================================================================================