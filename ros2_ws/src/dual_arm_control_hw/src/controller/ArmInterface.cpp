#include"controller/ArmInterface.h"

//切换到笛卡尔空间控制时，保持输出切换前关节角度/角速度的轮数
constexpr int SWITCH_HOLD_TIMES = 20;

//==================================================

/*基础操作封装*/

//重力补偿封装

void DualArmGravWrap::init_GravComp(
    //重力补偿参量
    std::vector<double> waist_stPos,
    KDL::JntArray lArm_stPos, KDL::JntArray rArm_stPos
){
    //控制参量刷新：从全局重力补偿配置中读取开关量
    this->ifUseInpFtr  = robotGravCompInte.ifUseInpFtr;
    this->ifUseOutpFtr = robotGravCompInte.ifUseOutpFtr;

    mForceComp_floatBase.init(waist_stPos, lArm_stPos, rArm_stPos);

    //滤波器初始化 —— 从 robotGravCompInte.gravComp_InpOutpFtr 读取参数
    const auto& ftr = robotGravCompInte.gravComp_InpOutpFtr;

    // resize 滤波器数组
    posInp_lowpassFilter_lArm.resize(LARMDOF);
    velInp_lowpassFilter_lArm.resize(LARMDOF);
    TorInp_lowpassFilter_lArm.resize(LARMDOF);
    grav_TorOutp_lowpassFilter_lArm.resize(LARMDOF);
    Inte_TorOutp_lowpassFilter_lArm.resize(LARMDOF);
    feedforward_TorComp_lowpassFilter_lArm.resize(LARMDOF);

    posInp_lowpassFilter_rArm.resize(RARMDOF);
    velInp_lowpassFilter_rArm.resize(RARMDOF);
    TorInp_lowpassFilter_rArm.resize(RARMDOF);
    grav_TorOutp_lowpassFilter_rArm.resize(RARMDOF);
    Inte_TorOutp_lowpassFilter_rArm.resize(RARMDOF);
    feedforward_TorComp_lowpassFilter_rArm.resize(RARMDOF);

    for(int i = 0; i < LARMDOF; i++){
        // 左臂：位置输入滤波器
        posInp_lowpassFilter_lArm[i].init(1.0 / TEMPTIME, ftr.lArm_posInpFtr[i].cutoffRate);
        // 左臂：速度输入滤波器
        velInp_lowpassFilter_lArm[i].init(1.0 / TEMPTIME, ftr.lArm_velInpFtr[i].cutoffRate);
        // 左臂：力矩输入滤波器
        TorInp_lowpassFilter_lArm[i].init(1.0 / TEMPTIME, ftr.lArm_torInpFtr[i].cutoffRate);
        // 左臂：力矩输出滤波器（重力/惯性/前馈共用 torOutpFtr）
        grav_TorOutp_lowpassFilter_lArm[i].init(1.0 / TEMPTIME, ftr.lArm_torOutpFtr[i].cutoffRate);
        Inte_TorOutp_lowpassFilter_lArm[i].init(1.0 / TEMPTIME, ftr.lArm_torOutpFtr[i].cutoffRate);
        feedforward_TorComp_lowpassFilter_lArm[i].init(1.0 / TEMPTIME, ftr.lArm_torOutpFtr[i].cutoffRate);

        // 右臂：位置输入滤波器
        posInp_lowpassFilter_rArm[i].init(1.0 / TEMPTIME, ftr.rArm_posInpFtr[i].cutoffRate);
        // 右臂：速度输入滤波器
        velInp_lowpassFilter_rArm[i].init(1.0 / TEMPTIME, ftr.rArm_velInpFtr[i].cutoffRate);
        // 右臂：力矩输入滤波器
        TorInp_lowpassFilter_rArm[i].init(1.0 / TEMPTIME, ftr.rArm_torInpFtr[i].cutoffRate);
        // 右臂：力矩输出滤波器
        grav_TorOutp_lowpassFilter_rArm[i].init(1.0 / TEMPTIME, ftr.rArm_torOutpFtr[i].cutoffRate);
        Inte_TorOutp_lowpassFilter_rArm[i].init(1.0 / TEMPTIME, ftr.rArm_torOutpFtr[i].cutoffRate);
        feedforward_TorComp_lowpassFilter_rArm[i].init(1.0 / TEMPTIME, ftr.rArm_torOutpFtr[i].cutoffRate);
    }
}

TorCompWrap DualArmGravWrap::GravCompOutp(
    //输入量
    std::vector<double> waist_pos, std::vector<double> waist_vel,       //基座位置、速度
    std::vector<double> lArm_pos, std::vector<double> lArm_vel,
    std::vector<double> rArm_pos, std::vector<double> rArm_vel,
    const std::vector<double>& acceleration
){  
    std::vector<double> torComp_grav;           //重力补偿输出
    std::vector<double> torComp_InteriaComb;    //惯性补偿输出
    std::vector<double> torFeedForward;         //力前馈输出，这里仅仅包含重力补偿模型中自带的摩擦模型

    std::vector<double> fricComp;       //接受辅助参数 -- 用于接受摩擦补偿输出

    //更新基座位置、速度
    mForceComp_floatBase.baseStateUpdate(
        waist_pos, waist_vel
    );

    if(ifUseInpFtr == 1 && acceleration.empty()){
        //对输入进行滤波处理
        std::vector<double> lArm_posRetFtred;
        std::vector<double> lArm_velRetFtred;

        std::vector<double> rArm_posRetFtred;
        std::vector<double> rArm_velRetFtred;

        for(int i = 0; i < LARMDOF; i++){

            lArm_posRetFtred.push_back(posInp_lowpassFilter_lArm[i].filter(lArm_pos.at(i)));
            lArm_velRetFtred.push_back(velInp_lowpassFilter_lArm[i].filter(lArm_vel.at(i)));

            rArm_posRetFtred.push_back(posInp_lowpassFilter_rArm[i].filter(rArm_pos.at(i)));
            rArm_velRetFtred.push_back(velInp_lowpassFilter_rArm[i].filter(rArm_vel.at(i)));

        }
        torComp_grav = mForceComp_floatBase.gravComp(lArm_posRetFtred, lArm_velRetFtred,
                                        rArm_posRetFtred, rArm_velRetFtred);
        torComp_InteriaComb = mForceComp_floatBase.InteriaMulComp(
            lArm_posRetFtred, lArm_velRetFtred,
            rArm_posRetFtred, rArm_velRetFtred
        );


        //前馈量赋值
        torFeedForward.assign(LARMDOF+RARMDOF, 0.0);

    }else{

        //重力补偿赋值
        torComp_grav = mForceComp_floatBase.gravComp(lArm_pos, lArm_vel,
                                    rArm_pos, rArm_vel);

        //惯性综合补偿赋值
        torComp_InteriaComb = mForceComp_floatBase.InteriaMulComp(
            lArm_pos, lArm_vel,
            rArm_pos, rArm_vel, acceleration
        ); 
        

        //摩擦补偿输出
        torFeedForward.assign(LARMDOF+RARMDOF, 0.0);
    }

    //对输出进行滤波处理
    std::vector<double> grav_TorComp_Ftred;         //重力补偿（滤波后）
    std::vector<double> Intertia_TorComp_Ftred;     //惯性补偿（滤波后）
    std::vector<double> FeedForward_TorComp_Ftred;  //摩擦前馈补偿（滤波后）


    if(ifUseOutpFtr == 1){
        int m_temp = 0;
        for(int i = 0; i < LARMDOF; i++){
            grav_TorComp_Ftred.push_back(grav_TorOutp_lowpassFilter_lArm[i].filter(torComp_grav.at(m_temp)));
            Intertia_TorComp_Ftred.push_back(Inte_TorOutp_lowpassFilter_lArm[i].filter(torComp_InteriaComb.at(m_temp)));
            FeedForward_TorComp_Ftred.push_back(feedforward_TorComp_lowpassFilter_lArm[i].filter(torFeedForward.at(m_temp)));
            m_temp += 1;
        }
        for(int j = 0; j < RARMDOF; j++){
            grav_TorComp_Ftred.push_back(grav_TorOutp_lowpassFilter_rArm[j].filter(torComp_grav.at(m_temp)));
            Intertia_TorComp_Ftred.push_back(Inte_TorOutp_lowpassFilter_rArm[j].filter(torComp_InteriaComb.at(m_temp)));
            FeedForward_TorComp_Ftred.push_back(feedforward_TorComp_lowpassFilter_rArm[j].filter(torFeedForward.at(m_temp)));
            m_temp += 1;
        }
    }else{
        grav_TorComp_Ftred = torComp_grav;
        Intertia_TorComp_Ftred = torComp_InteriaComb;
        FeedForward_TorComp_Ftred = torFeedForward;
    }

    //结果输出包装
    TorCompWrap mRes;
    mRes.lArm_Comp_grav.assign(LARMDOF, 0.0);
    mRes.rArm_comp_grav.assign(RARMDOF, 0.0);

    mRes.lArm_Comp_InteriaComb.assign(LARMDOF, 0.0);
    mRes.rArm_Comp_InteriaComb.assign(RARMDOF, 0.0);

    mRes.lArm_feedForward.assign(LARMDOF, 0.0);
    mRes.rArm_feedForward.assign(RARMDOF, 0.0);

    for(int i = 0; i < LARMDOF; i++){
        mRes.lArm_Comp_grav[i] = grav_TorComp_Ftred[i];
        mRes.rArm_comp_grav[i] = grav_TorComp_Ftred[i + LARMDOF];

        mRes.lArm_Comp_InteriaComb[i] = Intertia_TorComp_Ftred[i];
        mRes.rArm_Comp_InteriaComb[i] = Intertia_TorComp_Ftred[i + LARMDOF];

        mRes.lArm_feedForward[i] = FeedForward_TorComp_Ftred[i];
        mRes.rArm_feedForward[i] = FeedForward_TorComp_Ftred[i + LARMDOF];
    }

    return mRes;
}

//纯粹的重力补偿函数——输出中仅包含重力补偿
TorCompWrap DualArmGravWrap::onlyGravComp(
    //输入量：
    std::vector<double> waist_pos, std::vector<double> waist_vel,
    std::vector<double> lArm_pos, std::vector<double> lArm_vel,
    std::vector<double> rArm_pos, std::vector<double> rArm_vel
){
    std::vector<double> torComp_grav;           //重力补偿输出

    //更新基座位置、速度
    mForceComp_floatBase.baseStateUpdate(
        waist_pos, waist_vel
    );

    if(ifUseInpFtr == 1){
        //对输入进行滤波处理
        std::vector<double> lArm_posRetFtred;
        std::vector<double> lArm_velRetFtred;

        std::vector<double> rArm_posRetFtred;
        std::vector<double> rArm_velRetFtred;

        for(int i = 0; i < LARMDOF; i++){

            lArm_posRetFtred.push_back(posInp_lowpassFilter_lArm[i].filter(lArm_pos.at(i)));
            lArm_velRetFtred.push_back(velInp_lowpassFilter_lArm[i].filter(lArm_vel.at(i)));

            rArm_posRetFtred.push_back(posInp_lowpassFilter_rArm[i].filter(rArm_pos.at(i)));
            rArm_velRetFtred.push_back(velInp_lowpassFilter_rArm[i].filter(rArm_vel.at(i)));
        }

        torComp_grav = mForceComp_floatBase.gravComp(lArm_posRetFtred, lArm_velRetFtred,
                                        rArm_posRetFtred, rArm_velRetFtred);


        reserve[0] = lArm_posRetFtred.at(1);
        reserve[1] = rArm_posRetFtred.at(1);

    }else{

        //重力补偿赋值
        torComp_grav = mForceComp_floatBase.gravComp(lArm_pos, lArm_vel,
                                    rArm_pos, rArm_vel);

    }

    //对输出进行滤波处理
    std::vector<double> grav_TorComp_Ftred;         //重力补偿（滤波后）


    if(ifUseOutpFtr == 1){
        int m_temp = 0;
        for(int i = 0; i < LARMDOF; i++){
            grav_TorComp_Ftred.push_back(grav_TorOutp_lowpassFilter_lArm[i].filter(torComp_grav.at(m_temp)));
            m_temp += 1;
        }
        for(int j = 0; j < RARMDOF; j++){
            grav_TorComp_Ftred.push_back(grav_TorOutp_lowpassFilter_rArm[j].filter(torComp_grav.at(m_temp)));
            m_temp += 1;
        }
    }else{
        grav_TorComp_Ftred = torComp_grav;
    }

    //结果输出包装
    TorCompWrap mRes;

    mRes.lArm_Comp_grav.assign(LARMDOF, 0.0);
    mRes.rArm_comp_grav.assign(RARMDOF, 0.0);

    mRes.lArm_Comp_InteriaComb.assign(LARMDOF, 0.0);
    mRes.rArm_Comp_InteriaComb.assign(RARMDOF, 0.0);

    mRes.lArm_feedForward.assign(LARMDOF, 0.0);
    mRes.rArm_feedForward.assign(RARMDOF, 0.0);

    for(int i = 0; i < LARMDOF; i++){
        mRes.lArm_Comp_grav[i] = grav_TorComp_Ftred[i];
        mRes.rArm_comp_grav[i] = grav_TorComp_Ftred[i + LARMDOF];
    }

    return mRes;
}

//PD参数初始化 -- 从.yaml配置文件读取PD参数
void PDInte::PDInteInit(){
    // 调整所有PD数组大小
    Kp_lArm_low.resize(LARMDOF);
    Kd_lArm_low.resize(LARMDOF);
    Kp_rArm_low.resize(RARMDOF);
    Kd_rArm_low.resize(RARMDOF);

    Kp_lArm_mid.resize(LARMDOF);
    Kd_lArm_mid.resize(LARMDOF);
    Kp_rArm_mid.resize(RARMDOF);
    Kd_rArm_mid.resize(RARMDOF);

    Kp_lArm_high.resize(LARMDOF);
    Kd_lArm_high.resize(LARMDOF);
    Kp_rArm_high.resize(RARMDOF);
    Kd_rArm_high.resize(RARMDOF);

    Kp_lArm_superhigh.resize(LARMDOF);
    Kd_lArm_superhigh.resize(LARMDOF);
    Kp_rArm_superhigh.resize(RARMDOF);
    Kd_rArm_superhigh.resize(RARMDOF);

    Kp_lArm_tag.resize(LARMDOF);
    Kd_lArm_tag.resize(LARMDOF);
    Kp_rArm_tag.resize(RARMDOF);
    Kd_rArm_tag.resize(RARMDOF);

    // 从YAML全局变量拷贝PD参数 (double → double)
    for(int i = 0; i < LARMDOF; i++){
        // 低刚度 (low)
        Kp_lArm_low[i] = static_cast<double>(lArm_defaultKp_low[i]);
        Kd_lArm_low[i] = static_cast<double>(lArm_defaultKd_low[i]);
        Kp_rArm_low[i] = static_cast<double>(rArm_defaultKp_low[i]);
        Kd_rArm_low[i] = static_cast<double>(rArm_defaultKd_low[i]);

        // 中等刚度 (mid)
        Kp_lArm_mid[i] = static_cast<double>(lArm_defaultKp_mid[i]);
        Kd_lArm_mid[i] = static_cast<double>(lArm_defaultKd_mid[i]);
        Kp_rArm_mid[i] = static_cast<double>(rArm_defaultKp_mid[i]);
        Kd_rArm_mid[i] = static_cast<double>(rArm_defaultKd_mid[i]);

        // 高刚度 (high)
        Kp_lArm_high[i] = static_cast<double>(lArm_defaultKp_high[i]);
        Kd_lArm_high[i] = static_cast<double>(lArm_defaultKd_high[i]);
        Kp_rArm_high[i] = static_cast<double>(rArm_defaultKp_high[i]);
        Kd_rArm_high[i] = static_cast<double>(rArm_defaultKd_high[i]);

        // 超高刚度 (superhigh)
        Kp_lArm_superhigh[i] = static_cast<double>(lArm_defaultKp_superhigh[i]);
        Kd_lArm_superhigh[i] = static_cast<double>(lArm_defaultKd_superhigh[i]);
        Kp_rArm_superhigh[i] = static_cast<double>(rArm_defaultKp_superhigh[i]);
        Kd_rArm_superhigh[i] = static_cast<double>(rArm_defaultKd_superhigh[i]);

        // 拖拽模式 (tag) — Kp全0, Kd全12
        Kp_lArm_tag[i] = static_cast<double>(lArm_defaultKp_tag[i]);
        Kd_lArm_tag[i] = static_cast<double>(lArm_defaultKd_tag[i]);
        Kp_rArm_tag[i] = static_cast<double>(rArm_defaultKp_tag[i]);
        Kd_rArm_tag[i] = static_cast<double>(rArm_defaultKd_tag[i]);
    }

    last_rigidityChoose = 1;
    ifTrigger_smoothTrans = false;
    smoothTrans_tempCount = 0;

    resLast.lArmKp = Kp_lArm_low;
    resLast.lArmKd = Kd_lArm_low;
    resLast.rArmKp = Kp_rArm_low;
    resLast.rArmKd = Kd_rArm_low;

    std::cout << "PD参数已从YAML配置文件同步到PDInte" << std::endl;
}

//Kp，Kd输出封装
ArmPD PDInte::PDInteOutp(int modChoose, int rigidChoose){
    ArmPD mRes;
    if(modChoose == 0){
        //默认低刚度输出
        for(int i = 0; i < LARMDOF; i++){
            mRes.lArmKp.push_back(Kp_lArm_low[i]);
            mRes.lArmKd.push_back(Kd_lArm_low[i]);
            mRes.rArmKp.push_back(Kp_rArm_low[i]);
            mRes.rArmKd.push_back(Kd_rArm_low[i]);
        }
    }else if(modChoose == 5){
        //零力拖动模式
        for(int i = 0; i < LARMDOF; i++){
            mRes.lArmKp.push_back(Kp_lArm_tag[i]);
            mRes.lArmKd.push_back(Kd_lArm_tag[i]);
            mRes.rArmKp.push_back(Kp_rArm_tag[i]);
            mRes.rArmKd.push_back(Kd_rArm_tag[i]);
        }
    }
    else{
        if(rigidChoose == 1){
            //低刚度模式

            for(int i = 0; i < LARMDOF; i++){
                mRes.lArmKp.push_back(Kp_lArm_low[i]);
                mRes.lArmKd.push_back(Kd_lArm_low[i]);
                mRes.rArmKp.push_back(Kp_rArm_low[i]);
                mRes.rArmKd.push_back(Kd_rArm_low[i]);
            }
        }else if(rigidChoose == 2){
            //中等刚度模式

            for(int i = 0; i < LARMDOF; i++){
                mRes.lArmKp.push_back(Kp_lArm_mid[i]);
                mRes.lArmKd.push_back(Kd_lArm_mid[i]);
                mRes.rArmKp.push_back(Kp_rArm_mid[i]);
                mRes.rArmKd.push_back(Kd_rArm_mid[i]);
            }
        }else if(rigidChoose == 3){
            //高刚度模式

            for(int i = 0; i < LARMDOF; i++){
                mRes.lArmKp.push_back(Kp_lArm_high[i]);
                mRes.lArmKd.push_back(Kd_lArm_high[i]);
                mRes.rArmKp.push_back(Kp_rArm_high[i]);
                mRes.rArmKd.push_back(Kd_rArm_high[i]);
            }
        }
        else if(rigidChoose == 7){
            //超高刚度模式

            for(int i = 0; i < LARMDOF; i++){
                mRes.lArmKp.push_back(Kp_lArm_superhigh[i]);
                mRes.lArmKd.push_back(Kd_lArm_superhigh[i]);
                mRes.rArmKp.push_back(Kp_rArm_superhigh[i]);
                mRes.rArmKd.push_back(Kd_rArm_superhigh[i]);
            }
        } 
        else{
            //默认中刚度模式

            for(int i = 0; i < LARMDOF; i++){
                mRes.lArmKp.push_back(Kp_lArm_mid[i]);
                mRes.lArmKd.push_back(Kd_lArm_mid[i]);
                mRes.rArmKp.push_back(Kp_rArm_mid[i]);
                mRes.rArmKd.push_back(Kd_rArm_mid[i]);
            }
        }

        //刚度切换代码
        if(rigidChoose != last_rigidityChoose){
            ifTrigger_smoothTrans = true;
        }
        if(ifTrigger_smoothTrans){
            ArmPD mResDes = mRes;

            ArmPD derRes;
            derRes.lArmKd.assign(LARMDOF, 0.0);
            derRes.lArmKp.assign(LARMDOF, 0.0);
            derRes.rArmKd.assign(RARMDOF, 0.0);
            derRes.rArmKp.assign(RARMDOF, 0.0);

            for(int i = 0; i < LARMDOF; i++){
                //计算增量
                derRes.lArmKp[i] = (mRes.lArmKp[i] - resLast.lArmKp[i]) / static_cast<double>(smoothTrans_trajLen);
                derRes.lArmKd[i] = (mRes.lArmKd[i] - resLast.lArmKd[i]) / static_cast<double>(smoothTrans_trajLen);

                //计算输出值
                mRes.lArmKp[i] = resLast.lArmKp[i] + static_cast<double>(smoothTrans_tempCount) * derRes.lArmKp[i];
                mRes.lArmKd[i] = resLast.lArmKd[i] + static_cast<double>(smoothTrans_tempCount) * derRes.lArmKd[i];
            }
            for(int i = 0; i < RARMDOF; i++){
                //计算增量
                derRes.rArmKp[i] = (mRes.rArmKp[i] - resLast.rArmKp[i]) / static_cast<double>(smoothTrans_trajLen);
                derRes.rArmKd[i] = (mRes.rArmKd[i] - resLast.rArmKd[i]) / static_cast<double>(smoothTrans_trajLen);

                //计算输出值
                mRes.rArmKp[i] = resLast.rArmKp[i] + static_cast<double>(smoothTrans_tempCount) * derRes.rArmKp[i];
                mRes.rArmKd[i] = resLast.rArmKd[i] + static_cast<double>(smoothTrans_tempCount) * derRes.rArmKd[i];
            }

            smoothTrans_tempCount += 1;
            if(smoothTrans_tempCount == smoothTrans_trajLen || smoothTrans_tempCount > smoothTrans_trajLen){
                ifTrigger_smoothTrans = false;
                resLast = mResDes;

                smoothTrans_tempCount = 0;
                last_rigidityChoose = rigidChoose;
            }
        }
    }
    return mRes;
}

//================================================================================
/*辅助函数*/

/*  类型转换函数：
------------------
注意：Frame类型转化对应的vector容器应当是六维度向量，其数据含义依次为：
[x, y, z, row, pitch, yaw] -- 三个平移量+三个欧拉角
*/

// 将 std::vector<double> 转换为 KDL::JntArray
KDL::JntArray vec_to_JntArray(const std::vector<double>& mVecInp) {
    KDL::JntArray jntArray;
    if (!mVecInp.empty()) {
        jntArray.resize(mVecInp.size());
        for (size_t i = 0; i < mVecInp.size(); ++i) {
            jntArray(i) = mVecInp[i];
        }
    }
    return jntArray;
}

// 将 KDL::JntArray 转换为 std::vector<double>
std::vector<double> JntArray_to_vec(const KDL::JntArray& mArrInp) {
    std::vector<double> vec;
    if (mArrInp.rows() > 0) {
        vec.resize(mArrInp.rows());
        for (unsigned int i = 0; i < mArrInp.rows(); ++i) {
            vec[i] = mArrInp(i);
        }
    }
    return vec;
}

// 将 std::vector<double> 转换为 KDL::Frame（六维向量：[x, y, z, roll, pitch, yaw]）
KDL::Frame vec_to_Frame(const std::vector<double>& mVecInp) {
    if (mVecInp.size() != 6) {
        throw std::runtime_error("Input vector must have 6 elements for Frame conversion.");
    }

    // 提取平移部分
    KDL::Vector translation(mVecInp[0], mVecInp[1], mVecInp[2]);

    // 提取旋转部分（欧拉角转旋转矩阵）
    KDL::Rotation rotation = KDL::Rotation::RPY(mVecInp[3], mVecInp[4], mVecInp[5]);

    return KDL::Frame(rotation, translation);
}

// 将 KDL::Frame 转换为 std::vector<double>（六维向量：[x, y, z, roll, pitch, yaw]）
std::vector<double> Frame_to_vec(const KDL::Frame& mFrameInp) {
    std::vector<double> vec(6);

    // 提取平移部分
    vec[0] = mFrameInp.p.x();
    vec[1] = mFrameInp.p.y();
    vec[2] = mFrameInp.p.z();

    // 提取旋转部分（旋转矩阵转欧拉角）
    mFrameInp.M.GetRPY(vec[3], vec[4], vec[5]);

    return vec;
}

//================================================================================
/*整体封装*/

int sgnFunc(double mInp){
    if(mInp > 1e-7){
        return 1;
    }else if(mInp < -1*1e-7){
        return -1;
    }else{
        return 0;
    }
}
double corUpBound(double mInp, double upperbound, double lowerbound){
    double mRes;
    double tInp = mInp;
    if(mInp > upperbound){
        mRes = upperbound;
    }else if(mInp < lowerbound){
        mRes = lowerbound;
        /**
    }else if(std::abs(std::abs(tInp) - 1.5708) < 1e-4){
        if(tInp > 0){
            mInp = 1.5707;
        }else{
            mInp = -1.5707;
        }
        /**/
    }else{
        mRes = mInp;
    }
    return mRes;
}
KDL::Frame FrameCor(KDL::Frame FrameInp){
    double row, pitch, yaw;
    FrameInp.M.GetRPY(row, pitch, yaw);
    double upperBound = 3.14159;
    double lowerBound = -3.14159;
    
    KDL::Frame resFrame;
    resFrame.p[0] = FrameInp.p[0];
    resFrame.p[1] = FrameInp.p[1];
    resFrame.p[2] = FrameInp.p[2];
    double mRow = corUpBound(row, upperBound, lowerBound);
    double mPitch = corUpBound(pitch, upperBound, lowerBound);
    double mYaw = corUpBound(yaw, upperBound, lowerBound);
    resFrame.M = KDL::Rotation::RPY(mRow, mPitch, mYaw);

    return resFrame;
}


bool ArmCtlOptInterface::init(
    std::vector<double> waist_stPos,
    KDL::JntArray stPos_lArm, KDL::JntArray stPos_rArm,
    JntLimMsg lArm_LimJnt, JntLimMsg rArm_LimJnt,
    modCtl mod_ctl_msg,
    optiPrintMsgCtl mPrintMsgPkg,   //程序打印控制参数
    optimizerCtl m_opti_ctl_msg     //优化器参数
){
    this->stPos_l = stPos_lArm;
    this->stPos_r = stPos_rArm;
    this->waist_stPos_ = waist_stPos;
    this->outpMod = mod_ctl_msg.transMod;

    //模式切换平滑过渡初始化
    this->lastOutpMod = mod_ctl_msg.transMod;
    this->switchHoldCount = 0;


    //初始化关节控制模式
    JntSpace_track.init(
        stPos_lArm, stPos_rArm
    );

    //初始化笛卡尔空间轨迹求解模式
    std::vector<double> lArmStAngConf, rArmStAngConf;
    for(int i = 0; i < stPos_lArm.rows(); i++){
        lArmStAngConf.push_back(stPos_lArm(i));
    }
    for(int i = 0; i < stPos_rArm.rows(); i++){
        rArmStAngConf.push_back(stPos_rArm(i));
    }

    if (!FrameSpace_track.init_solver(
        lArm_LimJnt, rArm_LimJnt,
        lArmStAngConf, rArmStAngConf,
        mPrintMsgPkg,
        m_opti_ctl_msg
    )) {
        std::cerr << "FATAL: ArmCtlOptInterface::init() FAILED — "
                  << "FrameTrackSol::init_solver() returned false. "
                  << "The IK/FK solver wrapper could not be initialized." << std::endl;
        return false;
    }

    //初始化电流常数
    lArm_currentConstant = robotModelStructConfig.lArm_jnt_Kt;
    rArm_currentConstant = robotModelStructConfig.rArm_jnt_Kt;

    //初始化PD参数
    mArmPDInte.PDInteInit();

    //初始化关节角度/角速度输出低通滤波器
    ifUsePosVelOutpFtr = robotGravCompInte.ifUsePosVelOutpFtr;
    {
        const auto& outpFtr = robotGravCompInte.gravComp_InpOutpFtr;
        //参数维度校验：若配置缺失或维度不匹配，则关闭该滤波器，避免越界访问
        if(static_cast<int>(outpFtr.lArm_posOutpFtr.size()) != LARMDOF
           || static_cast<int>(outpFtr.lArm_velOutpFtr.size()) != LARMDOF
           || static_cast<int>(outpFtr.rArm_posOutpFtr.size()) != RARMDOF
           || static_cast<int>(outpFtr.rArm_velOutpFtr.size()) != RARMDOF){
            ifUsePosVelOutpFtr = 0;
        }
    }
    if(ifUsePosVelOutpFtr){
        const auto& outpFtr = robotGravCompInte.gravComp_InpOutpFtr;
        posOutp_lowpassFilter_lArm.resize(LARMDOF);
        velOutp_lowpassFilter_lArm.resize(LARMDOF);
        posOutp_lowpassFilter_rArm.resize(RARMDOF);
        velOutp_lowpassFilter_rArm.resize(RARMDOF);
        for(int i = 0; i < LARMDOF; i++){
            posOutp_lowpassFilter_lArm[i].init(1.0 / TEMPTIME, outpFtr.lArm_posOutpFtr[i].cutoffRate);
            velOutp_lowpassFilter_lArm[i].init(1.0 / TEMPTIME, outpFtr.lArm_velOutpFtr[i].cutoffRate);
            posOutp_lowpassFilter_rArm[i].init(1.0 / TEMPTIME, outpFtr.rArm_posOutpFtr[i].cutoffRate);
            velOutp_lowpassFilter_rArm[i].init(1.0 / TEMPTIME, outpFtr.rArm_velOutpFtr[i].cutoffRate);
        }
        //将滤波器初值设为初始关节位置，避免上电瞬间滤波收敛产生的跳变
        for(int i = 0; i < LARMDOF; i++){
            posOutp_lowpassFilter_lArm[i].reset(stPos_lArm(i));
            velOutp_lowpassFilter_lArm[i].reset(0.0);
        }
        for(int i = 0; i < RARMDOF; i++){
            posOutp_lowpassFilter_rArm[i].reset(stPos_rArm(i));
            velOutp_lowpassFilter_rArm[i].reset(0.0);
        }
    }

    //初始化过程参数
    rigidityMod_l.resize(LARMDOF);
    rigidityMod_r.resize(RARMDOF);
    for(int i = 0; i < LARMDOF; i++){
        rigidityMod_l[i] = mod_ctl_msg.base_rigiditiCtl_mod_l[i];
        rigidityMod_r[i] = mod_ctl_msg.base_rigiditiCtl_mod_r[i];
    }

    //所有的上一轮响应置零
    lArmLst_outpWrap.posDes.assign(LARMDOF, 0.0);              rArmLst_outpWrap.posDes.assign(RARMDOF, 0.0);
    lArmLst_outpWrap.velDes.assign(LARMDOF, 0.0);              rArmLst_outpWrap.velDes.assign(RARMDOF, 0.0);
    lArmLst_outpWrap.KpDes.assign(LARMDOF, 0.0);               rArmLst_outpWrap.KpDes.assign(RARMDOF, 0.0);
    lArmLst_outpWrap.KdDes.assign(LARMDOF, 0.0);               rArmLst_outpWrap.KdDes.assign(RARMDOF, 0.0);
    lArmLst_outpWrap.gravComp_TorDes.assign(LARMDOF, 0.0);     rArmLst_outpWrap.gravComp_TorDes.assign(RARMDOF, 0.0);
    lArmLst_outpWrap.FeedForward_TorDes.assign(LARMDOF, 0.0);  rArmLst_outpWrap.FeedForward_TorDes.assign(RARMDOF, 0.0);
    lArmLst_outpWrap.InteriaComp_TorDes.assign(LARMDOF, 0.0);  rArmLst_outpWrap.InteriaComp_TorDes.assign(RARMDOF, 0.0);
    for(int i = 0; i < LARMDOF; i++){
        lArmLst_outpWrap.posDes.at(i) = stPos_lArm(i);
        rArmLst_outpWrap.posDes.at(i) = stPos_rArm(i);
    }
    stPos_l = stPos_lArm;
    stPos_r = stPos_rArm;
    return true;
}

void ArmCtlOptInterface::Traj_cal(
    bool ifUpdateLim,
    JntLimMsg allArmLimJnt,

    //笛卡尔空间目标值
    KDL::Frame lArm_desPos_Frame, KDL::Frame rArm_desPos_Frame,
    std::vector<double> lArm_desTor_Frame, std::vector<double> rArm_desTor_Frame,

    //关节空间的目标值
    KDL::JntArray lArm_desPos_Jnt, KDL::JntArray rArm_desPos_Jnt,
    std::vector<double> lArm_desTor_Jnt, std::vector<double> rArm_desTor_Jnt,

    //反馈值
    KDL::JntArray lArm_posRet, KDL::JntArray rArm_posRet,
    KDL::JntArray lArm_velRet, KDL::JntArray rArm_velRet,
    std::vector<double> lArm_iq_ret, std::vector<double> rArm_iq_ret,

    modCtl mod_ctl_msg,
    bool ifRedefineJntTrajStab
){
    this->outpMod = mod_ctl_msg.transMod;

    //利用反馈电流计算电机力矩
    std::vector<double> JntTor_l, JntTor_r;
    for(int i = 0; i < LARMDOF; i++){
        JntTor_l.push_back(lArm_iq_ret[i] * lArm_currentConstant[i]);
    }
    for(int i = 0; i < RARMDOF; i++){
        JntTor_r.push_back(rArm_iq_ret[i] * rArm_currentConstant[i]);
    }

    //进行一轮控制计算
    if(mod_ctl_msg.transMod == 2 || mod_ctl_msg.transMod == 4){
        //采用点位控制模式：
        FrameSpace_track.FrameTrack(
            ifUpdateLim,
            allArmLimJnt,

            lArm_desPos_Frame, rArm_desPos_Frame,
            lArm_desTor_Frame, rArm_desTor_Frame,

            lArm_posRet, rArm_posRet,
            lArm_velRet, rArm_velRet,
            JntTor_l, JntTor_r,

            mod_ctl_msg
        );

        //刷新刚度模式
        for(int k = 0; k < LARMDOF; k++){
            rigidityMod_l[k] = mod_ctl_msg.base_rigiditiCtl_mod_l[k];
            rigidityMod_r[k] = mod_ctl_msg.base_rigiditiCtl_mod_r[k];
        }
    }else if(mod_ctl_msg.transMod == 1 || mod_ctl_msg.transMod == 3){
        //采用关节控制模式：
        JntSpace_track.JntTrajCal(
            lArm_desPos_Jnt, rArm_desPos_Jnt,
            lArm_desTor_Jnt, rArm_desTor_Jnt,

            lArm_posRet, rArm_posRet,
            lArm_velRet, rArm_velRet,
            JntTor_l, JntTor_r,

            mod_ctl_msg,
            ifRedefineJntTrajStab
        );

        //刷新刚度模式
        for(int k = 0; k < LARMDOF; k++){
            rigidityMod_l[k] = mod_ctl_msg.base_rigiditiCtl_mod_l[k];
            rigidityMod_r[k] = mod_ctl_msg.base_rigiditiCtl_mod_r[k];
        }
    }else if(mod_ctl_msg.transMod == 5){
        //零力拖动模式

        //这里，关节目标位置与目标速度均置零，PD参数采用零力拖动对应的PD参数，仅进行重力补偿相关计算
        
    }
    else{
        //不进行计算
    }
}

std::vector<outpWrap> ArmCtlOptInterface::Traj_temp_outp(
    std::vector<double> waist_pos, std::vector<double> waist_vel,
    KDL::JntArray lArm_posRet, KDL::JntArray rArm_posRet,
    KDL::JntArray lArm_velRet, KDL::JntArray rArm_velRet,
    std::vector<double> lArm_iq_Ret, std::vector<double> rArm_iq_Ret
){
    outpWrap lArm_outpWrap;
    outpWrap rArm_outpWrap;

    //利用反馈电流计算反馈力矩
    std::vector<double> lArm_TorRet, rArm_TorRet;
    for(int i = 0; i < LARMDOF; i++){
        lArm_TorRet.push_back(lArm_iq_Ret[i] * lArm_currentConstant[i]);
        rArm_TorRet.push_back(rArm_iq_Ret[i] * rArm_currentConstant[i]);
    }
    //位置控制量、PD控制量计算
    std::vector<std::vector<ctlOutp>> posCtlRes;
    ArmPD allArmPDoutp;
    if(this->outpMod == 2 || this->outpMod == 4){
        //采用点位控制模式
        posCtlRes = FrameSpace_track.IKsolveResOutp(lArm_TorRet, rArm_TorRet);
        allArmPDoutp = mArmPDInte.PDInteOutp(this->outpMod, rigidityMod_l[0]);

    }else if(this->outpMod == 1 || this->outpMod == 3){
        //采用关节轨迹控制模式 ----- 
        posCtlRes = JntSpace_track.solveResOutp(lArm_TorRet, rArm_TorRet);
        allArmPDoutp = mArmPDInte.PDInteOutp(this->outpMod, rigidityMod_l[0]);
    }else if(this->outpMod == 5){
        //零力拖动模式

        //这里，关节目标位置与目标速度均置零，PD参数采用零力拖动对应的PD参数，仅进行重力补偿相关计算

        posCtlRes.resize(2);
        posCtlRes[0].resize(LARMDOF);
        posCtlRes[1].resize(RARMDOF);

        for(int i = 0; i < LARMDOF; i++){
            posCtlRes[0][i].posDes = 0.0;
            posCtlRes[0][i].velDes = 0.0;
            posCtlRes[0][i].grav_TorDes = 0.0;
            posCtlRes[0][i].interia_TorDes = 0.0;
            posCtlRes[0][i].feedforward_TorDes = 0.0;
            posCtlRes[0][i].ifReachEnd = false;

            posCtlRes[1][i].posDes = 0.0;
            posCtlRes[1][i].velDes = 0.0;
            posCtlRes[1][i].grav_TorDes = 0.0;
            posCtlRes[1][i].interia_TorDes = 0.0;
            posCtlRes[1][i].feedforward_TorDes = 0.0;
            posCtlRes[1][i].ifReachEnd = false;
        }

        //获取PD参数
        allArmPDoutp = mArmPDInte.PDInteOutp(this->outpMod, 0);
    }
    else{
        //此时输出为空值
        allArmPDoutp = mArmPDInte.PDInteOutp(this->outpMod, rigidityMod_l[0]);
    }

    //输出解包
    if(this->outpMod != 0){
        for(int i = 0; i < posCtlRes[0].size(); i++){
            lArm_outpWrap.posDes.push_back(posCtlRes[0][i].posDes);
            lArm_outpWrap.velDes.push_back(posCtlRes[0][i].velDes);

            lArm_outpWrap.gravComp_TorDes.push_back(posCtlRes[0][i].grav_TorDes);
            lArm_outpWrap.InteriaComp_TorDes.push_back(posCtlRes[0][i].interia_TorDes);
            lArm_outpWrap.FeedForward_TorDes.push_back(posCtlRes[0][i].feedforward_TorDes);

            lArm_outpWrap.ifComToEnd.push_back(posCtlRes[0][i].ifReachEnd);
        }
        for(int i = 0; i < posCtlRes[1].size(); i++){
            rArm_outpWrap.posDes.push_back(posCtlRes[1][i].posDes);
            rArm_outpWrap.velDes.push_back(posCtlRes[1][i].velDes);

            rArm_outpWrap.gravComp_TorDes.push_back(posCtlRes[1][i].grav_TorDes);
            rArm_outpWrap.InteriaComp_TorDes.push_back(posCtlRes[1][i].interia_TorDes);
            rArm_outpWrap.FeedForward_TorDes.push_back(posCtlRes[1][i].feedforward_TorDes);

            rArm_outpWrap.ifComToEnd.push_back(posCtlRes[1][i].ifReachEnd);
        }

        lArm_outpWrap.KpDes = allArmPDoutp.lArmKp;
        lArm_outpWrap.KdDes = allArmPDoutp.lArmKd;

        rArm_outpWrap.KpDes = allArmPDoutp.rArmKp;
        rArm_outpWrap.KdDes = allArmPDoutp.rArmKd;

    }else{
        lArm_outpWrap.posDes = lArm_zeroPos;
        lArm_outpWrap.velDes.assign(LARMDOF, 0.0);

        lArm_outpWrap.gravComp_TorDes.assign(LARMDOF, 0.0);
        lArm_outpWrap.InteriaComp_TorDes.assign(LARMDOF, 0.0);
        lArm_outpWrap.FeedForward_TorDes.assign(LARMDOF, 0.0);

        lArm_outpWrap.ifComToEnd.assign(LARMDOF, false);

        rArm_outpWrap.posDes = rArm_zeroPos;
        rArm_outpWrap.velDes.assign(RARMDOF, 0.0);

        rArm_outpWrap.gravComp_TorDes.assign(RARMDOF, 0.0);
        rArm_outpWrap.InteriaComp_TorDes.assign(RARMDOF, 0.0);
        rArm_outpWrap.FeedForward_TorDes.assign(RARMDOF, 0.0);

        rArm_outpWrap.ifComToEnd.assign(RARMDOF, false);

        for(int i = 0; i < LARMDOF; i++){
            lArm_outpWrap.posDes.at(i) = stPos_l(i);
            rArm_outpWrap.posDes.at(i) = stPos_r(i);
        }

        lArm_outpWrap.KpDes = allArmPDoutp.lArmKp;
        lArm_outpWrap.KdDes = allArmPDoutp.lArmKd;

        rArm_outpWrap.KpDes = allArmPDoutp.rArmKp;
        rArm_outpWrap.KdDes = allArmPDoutp.rArmKd;
    }

    /**
    std::cout << "============================================" << std::endl;
    std::cout << "outpMod = " << this->outpMod << std::endl;
    std::cout << "gravComp_mod = " << this->GravCompInpChoose << std::endl;
    for(int i = 0; i < lArm_outpWrap.posDes.size(); i++){
        std::cout << "lArm_posDes = " << lArm_outpWrap.posDes[i] << std::endl;
    }
    for(int i = 0; i < lArm_outpWrap.velDes.size(); i++){
        std::cout << "lArm_velDes = " << lArm_outpWrap.velDes[i] << std::endl;
    }
    for(int i = 0; i < rArm_outpWrap.posDes.size(); i++){
        std::cout << "rArm_posDes = " << rArm_outpWrap.posDes[i] << std::endl;
    }
    for(int i = 0; i < rArm_outpWrap.velDes.size(); i++){
        std::cout << "rArm_velDes = " << rArm_outpWrap.velDes[i] << std::endl;
    }
    std::cout << "============================================" << std::endl;
    /**/

    //如果关节静止，则要强制刷掉所有输出

    //模式切换平滑过渡 -- 切换到笛卡尔空间控制时，前50轮保持输出切换前关节角度/角速度
    {
        const bool ifCartesianNow = (this->outpMod == 2 || this->outpMod == 4);
        const bool ifCartesianLast = (lastOutpMod == 2 || lastOutpMod == 4);
        if(ifCartesianNow){
            if(!ifCartesianLast){
                //检测到切换到笛卡尔空间控制，记录切换前关节角度/角速度
                lArm_switchHoldPos = lArmLst_outpWrap.posDes;
                rArm_switchHoldPos = rArmLst_outpWrap.posDes;
                lArm_switchHoldVel = lArmLst_outpWrap.velDes;
                rArm_switchHoldVel = rArmLst_outpWrap.velDes;
                switchHoldCount = SWITCH_HOLD_TIMES;
                std::cout << "[ModeSwitch] 切换到笛卡尔空间控制模式 (outpMod=" << this->outpMod
                          << ")，前" << SWITCH_HOLD_TIMES << "轮保持输出切换前关节角度" << std::endl;
            }
            if(switchHoldCount > 0){
                //保持输出切换前关节角度/角速度
                lArm_outpWrap.posDes = lArm_switchHoldPos;
                rArm_outpWrap.posDes = rArm_switchHoldPos;
                lArm_outpWrap.velDes = lArm_switchHoldVel;
                rArm_outpWrap.velDes = rArm_switchHoldVel;
                switchHoldCount--;
            }
        }else{
            switchHoldCount = 0;    //离开笛卡尔模式，清除保持计数
        }
        lastOutpMod = this->outpMod;
    }

    //关节角度/角速度输出低通滤波
    if(ifUsePosVelOutpFtr){
        for(int i = 0; i < LARMDOF; i++){
            lArm_outpWrap.posDes[i] = posOutp_lowpassFilter_lArm[i].filter(lArm_outpWrap.posDes[i]);
            lArm_outpWrap.velDes[i] = velOutp_lowpassFilter_lArm[i].filter(lArm_outpWrap.velDes[i]);
        }
        for(int i = 0; i < RARMDOF; i++){
            rArm_outpWrap.posDes[i] = posOutp_lowpassFilter_rArm[i].filter(rArm_outpWrap.posDes[i]);
            rArm_outpWrap.velDes[i] = velOutp_lowpassFilter_rArm[i].filter(rArm_outpWrap.velDes[i]);
        }
    }

    //过程量更新
    lArmLst_outpWrap = lArm_outpWrap;
    rArmLst_outpWrap = rArm_outpWrap;

    std::vector<outpWrap> mRes = {lArm_outpWrap, rArm_outpWrap};

    return mRes;
}
