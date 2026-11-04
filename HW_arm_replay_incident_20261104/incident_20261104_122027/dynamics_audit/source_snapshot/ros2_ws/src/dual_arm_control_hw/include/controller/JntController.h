#ifndef __KERNEL_JntController__H__
#define __KERNEL_Jntcontroller__H__


#include <vector>
#include <numeric>
#include <complex>
#include <iomanip>
#include <algorithm>
#include <string>
#include <cstring>
#include <stdio.h>

#include <cstdlib>
#include <ctime>

#include <iostream>
#include <stdlib.h>
#include <string.h>
//#include <yaml-cpp/yaml.h>
//#include <control_toolbox/filters.h>
#include <fstream>
#include <cmath>
#include <time.h>
#include <sys/time.h>
#include "robot_config/config.h"
/*
        by 霜云喵
================================
单关节控制代码，用于单关节的力-阻抗控制
============修改日志============

================================
*/

namespace Kernel
{
    void mMatrixMul(double a[],double b[],int m,int n,int k,double c[]);
    #define MARMDOF (1) //定义全局变量
    #define PI2180 (0.017453)
    //======================MyController========================
    //单关节控制器

    /*notice: when testing,change all the "private:" into "public:" in order to make checking the datas more convenient*/

    //================================================================================
    /*      基本模块        */
    //定义PID控制器可插入模块
    class PIDcontrolBar
    {
        //private:
        public:
            //PID控制参数
            double mKp;
            double mKi;
            double mKd;
            double mKb; //积分反馈抑制抗饱和系数

            //PID内部运算参数
            double IntegerErrAccu;   //误差累加量
            double preErr;   //前项误差
            double preKbRet; //Kb反馈项
            double dertaT;   //单轮控制的时间

            //PID controller update

            //1. FeedForward link
            double mKf;

            //paraints for the system
            double mJMotor;
            double mKtMotor;
            double preDesireNum;
            double preDesireNumDt;



        //public:
            double temp; //迭代次数

            void CotrollerInit(double Kp, double Ki, double Kd, double KbInteigar, double mT); //输入量：Kp; Ki; Kd; Kb系数; 单轮控制时间
            void ControllerInitSimp(double Kp, double Ki, double Kd){
                this->mKp = Kp; this->mKi = Ki; this->mKd = Kd;
            }
            double ControlResultCal(double errNum);    //输入量：目标值减去传感器返回值的偏差量

            //PID controller update
            void FeedForwardInit(double Kf, double JMotor, double KtMotor);
            double ControlResultCalWithFeedForward(double errNum, double desireNum);

    };

    //定义低通滤波器可插入模块
    const double PI = 3.14159265358979323846f;
    #define BLOCKSIZE (256)  //滤波依赖的数据数量
    class Filter
    {
        private:
                double outvar[2];   //卡尔曼滤波器输出
                double Pmtrx[2][2]; //状态估计误差协方差矩阵
                double Qgain;   //噪声增益（用于调整过程噪声协方差矩阵Q的影响，该项越大滤波越强）
                double Rgain;   //测量噪声增益（用于调整测量噪声协方差矩阵的影响）
                
                //初始化状态估计协方差矩阵
                double P0mtrx[2][2]={{1000000.0f, 0.0f}, {0.0f, 1000000.0f}};
                //状态转移矩阵及其转置
                double FAImtrx[2][2] = {{1.0, 0.001}, {0.0, 1.0}};
                double FAImtrxT[2][2] = {{1.0,0.0}, {0.001, 1.0}};
                //过程噪声协方差矩阵
                double Qmtrx[2][2] = {{0.001*0.001*0.001/3.0f, 0.001*0.001*0.5f}, {0.001*0.001*0.5f, 0.001}};

                //中间变量
                double midx[2];  //预测的状态值
                double tempMtrx[2][2]; //临时矩阵
                double FAIPFAIT[2][2]; // 预测的状态估计误差协方差矩阵
                double Pk_[2][2]; // 更新后的状态估计误差协方差矩阵
                double Kgain[2]; // 卡尔曼增益

        public:
            void FilterInit(double inputVar, double mQgain, double mRgain);     //初始化函数
            double FilteredResCal(double inputvar);      //滤波结果计算函数
            //以下是老版本滤波器计算函数，对比来看新版本效果更好
            double FilteredResCaloldVersion(double inputvar); //老版本滤波器计算函数

    };

    class lowpassFilter
    {
        //传统的一阶低通滤波器
        private:
            double alpha;       // 滤波系数
            double lastOutput;  // 上一次输出值
            double sampleRate; // 采样频率（Hz）
            double timeConstant; // 时间常数（τ）(通过调节时间常数调节截止频率)
        public:

            void init(double sampleRate, double cutoffFreq);    //初始化函数
            double filter(double input);            //滤波处理函数
            void reset(double initValue = 0.0){     //重置函数
                this->lastOutput = initValue;
            }

    };

    //摩擦补偿模块
    class StribeckFriction
    {
        private:
            //参数定义：
            //摩擦参数：
            double fs;       //静摩擦系数
            double fc;       //库伦摩擦系数
            double fstic;    //粘滞摩擦系数

            double vs;       //临界速度
            double gamma;    //Stribeck系数（取0.5~2.0）

            double mTo;      //空载摩擦损耗
        public:
            void StribeckModelInit(double mfs, double mfc, double mfstic, double vs, double gama, double T0); //初始化函数
            double StribeckFricCal(double Tinp, double vel);   //计算Stribeck摩擦力的函数
    };
   
    //关节控制器力环
    //阻抗控制器
    class ImpendanceCtl
    {
        //阻抗控制参数计算模块
        //用途：将机器人关节等效为一个阻尼弹簧质量块模型，并用于计算要实现这种效果机器人关节的运动与出力参数

        public:
        //private:
            //阻抗控制参数
            double M_ImpCtl;     //质量参数
            double B_ImpCtl;     //阻尼参数
            double K_ImpCtl;     //刚度参数
            bool IfUseM_Inp;    //判断量：是否采用质量参数

            double dertaT;       //单轮控制时间

            double theta_0;      //理想目标位置
            double theta;        //理想实际位置
            double derta_theta;  //角度偏差量
            double Tor;          //负载大小

            int ctlMode;        //控制模式：0--非控制模式

            //解微分方程模块
            //定义模型对应的微分方程
            double dv0, dx0;   //定义迭代量：derta_theta的一阶导以及二阶导
            double h = 0.1;  //步长
            int steps = 50;    //计算步数
            double smallest_dertaRate = 0.005;
            void dervatives_Full(double x, double v, double &dx_dt, double &dv_dt, double F){
                //包含质量项的二阶微分方程，将其转化为两个一阶微分方程
                dv_dt = (F - this->B_ImpCtl*v - this->K_ImpCtl*x)/this->M_ImpCtl;
                dx_dt = v;
            }
            void dervatives_Part(double x, double &dx_dt, double F){
                //不包含质量项的二阶微分方程
                dx_dt = (F - this->K_ImpCtl*x)/this->B_ImpCtl;
            }
            double k1_x, k1_v;   //四阶龙格-库塔法系数
            double k2_x, k2_v;
            double k3_x, k3_v;
            double k4_x, k4_v;

        //public:
            //初始化函数
            void InitFunc(double B, double K, bool ifUseM, double dertaT, double start_Theta, double M = 0);     //初始化函数
            void updateInte(double B, double K, double M = 0);     //参数更新函数（搭配自适应控制使用）
        
            //基本操作函数
            void CalDertaByTor(double TorInp);      //利用输入力计算位置变化量(没有速度、加速度限制)（逆向阻抗）
            void CalTorByDerta(double derta_x, double derta_v, double derta_a = 0);    //利用位置变化量相关参数计算力（正向阻抗）

            //结果输出函数
            double ResOutp(int choose){
                if(choose == 1){  return this->derta_theta;   //采用力计算位置变化量
                }else if(choose == 2){  return this->Tor;     //返回计算得到的力修正量
                }else{  std::cout<< "Warning: choose num error." <<std::endl;
                    return 0.0f;
                }
            }
    };

    class ForceControl
    {
        //力控制器（仅包含力控制）
        public:
            PIDcontrolBar mForceCtl;    //电机PID控制器
            int ifNeedFilter;           //判断量：是否需要滤波器    1——需要滤波器
            Filter mForceInpFilter;     //力控制器的滤波器

            double ForceCtlOutp(double Inp);
    };

    //==============================================================================================================

    //关节控制类
    class JntCtl_TorImp
    {
        //单关节力-阻抗控制器
        private:
            PIDcontrolBar mForceControl;    //力控制器
            ImpendanceCtl mImpCtl;          //阻抗控制器

            int mT = 0.001;             //控制时间
            double Kt;                   //电流常数
            double Kt_inte;              //电流-力矩直线斜率
            int TorCompMod;             //开关量：力补偿模式 1--电流反馈；2--力直接反馈
            
            //关节限位
            double posLim[2];            //位置限制
            double velLim[2];            //速度限制
            double OutpLim[2];           //输出量大小限制

            //控制过程量
            double velDesLst;

        public:
            bool ifStCtl = false;        //开关量：是否开始控制，若不开始则强制输出量(不含补偿量部分)为0
            void init(double B, double K, bool ifUseM, double Start_Theta,
                      double torKp, double torKi, double torKd, double torKbInte,
                      double mKt, double mKtInte, int mTorCompMod,
                      double mposLim[], double mvelLim[], double mOutpLim[],
                      double M = 0
                     );
            double ctlOutp(double posDes, double posRet, double velDes, double IqRet, double OtherTorComp = 0);
    };

    class JntCtl_PDfeedforward
    {
        //单关节控制器--PD+力前馈控制器
        private:
            PIDcontrolBar mPosPDctl;        //位置环PD控制器
            PIDcontrolBar mVelPDctl;        //速度环PD控制器

            bool ifFirstTemp = true;
            bool ifUseTorComp;          //开关量：是否使用关节反馈力矩补偿
            int TorCompMod;             //开关量：力补偿模式 1--电流反馈；2--力直接反馈
            double Kt;                   //电流常数
            double KtInte;               //电流-力矩直线斜率
            double mT;           //控制周期为1ms

            //关节限位
            //协议：标号为0 -- 上限；标号为1 -- 下限
            double posLim[2];
            double velLim[2];
            double outpLim[2];
        public:
            void init(double posKp, double posKi, double posKd, double posKbInte, 
                      double velKp, double velKi, double velKd, double velKbInte,
                      double mposLim[], double mvelLim[], double moutpLim[],
                      int mTorCompMod,
                      bool mIfUseTorComp, double mKt, double mKtInte = 0);        //初始化函数
            double CtlOutp(double posDes, double IqRet = 0, double posRet = 0, double velRet = 0, double OtherTorComp = 0);   //输出函数(带力前馈)
    };
    int16_t dataTrans(double outp_f);
}


#endif