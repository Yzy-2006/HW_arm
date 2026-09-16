import numpy as np
import sys
from dataclasses import dataclass, field
from typing import List
from scipy.spatial.transform import Rotation

import os
import csv

# Use the Python environment selected at build time; optional vendor installation.
_openrobots_site = os.environ.get("OPENROBOTS_PYTHON_PATH")
if _openrobots_site:
    sys.path.insert(0, _openrobots_site)

import meshcat.geometry as mg
import pinocchio as pin                           
from pinocchio.visualize import MeshcatVisualizer
import casadi
from pinocchio import casadi as cpin


'''
        by 霜云喵
-------------------------
逆解求解封装模块
-------------------------
协议：
1. 目标位姿输入值是相对于arm_base_link的位姿

2. 所有角度均采用弧度制表示

3. 初始化过程中需要输入机器人当前的角度，如果开启角度限制，则限制关节定死的角度就是初始化的角度

4. 机器人关节顺序不能乱，必须遵从从左臂到右臂，关节ID从小到大的顺序，否则会出bug

-------------------------
修改：
1. 将位置限制的过程挪到DualArmIK类的初始化中进行，并将初始模型以及位置限制后的模型分别进行包装
2. 修改初始角度
3. 修改优化机制，添加速度连续性指标并添加相应初始化代码

'''

#利用casadi做的逆解求解器
@dataclass
class RobotModelDataWrap:
    #python数据包装类，用于模型数据信息包装与传输
    model_path: str      #原始机器人模型路径包装
    ArmEndLabel: List[str]              #机器人末端标签

@dataclass
class RobotModelLim:
    #python数据包装类，用于限制机器人特定关节的角度
    ifUseLim: bool      #判断量：是否使用位置限制
    LimJntLabelList: List[str]      #受限关节标签的列表
    ifDispJntMsg: bool  #开关量：是否打印关节信息

    LimJntAngList: List[float] = field(default_factory=list)      #受限关节角度的列表

    def __post_init__(self):
        #针对列表在初始化之后填充默认值：
        if not self.LimJntAngList:  #若用户未输入，则设定初始值
            self.LimJntAngList = []     #默认为空列表

@dataclass
class OptiInte:
    #python数据包装类，用于设定优化目标函数信息

    #优化过程基坐标系与末端坐标系
    baseFrameID: int = 3
    lArmEndID: int = 21
    rArmEndID: int = 39

    #优化器权重设置
    optiWeight: List[float] = field(default_factory=list)       #优化权重设置

    #滤波器参数设置
    FtrWeight: List[float] = field(default_factory=list)        #滤波器权重
    FtrDataLen: int = 14                                        #滤波器数据队列长度

    #优化器控制量：
    max_iter: int = 500         #最大迭代次数
    tol: float = 1e-5           #收敛容差度
    print_level: int = 0        #开关量——是否打印优化日志（默认不打印）
    print_time: bool = False    #开关量——是否打印计算时间（默认不打印）

    #优化过程的迭代时间(单位：s)
    tempTime: float = 0.02

    def __post_init__(self):
        #在初始化之后填充默认值
        if not self.optiWeight:     #若用户未输入，则设定为初始值
            self.optiWeight = [50.0, 1.0, 0.002, 0.01, 0.0002]
        if not self.FtrWeight:
            self.FtrWeight = [0.4, 0.3, 0.2, 0.1]
        # tol 合法性检查：IPOPT 要求 tol > 0，最小值为 1e-08
        # 若传入 tol <= 0（如未设置导致默认0），则使用安全默认值 1e-5
        if self.tol <= 0 or self.tol < 1e-8:
            print(f"Warning(IK-tol): tol={self.tol} is invalid for IPOPT (requires > 1e-08). "
                  f"Using default tol=1e-5 instead.")
            self.tol = 1e-5

@dataclass
class ViewMsg:
    #python数据包装类，用于可视化信息传输
    ifVisu: bool = False                #是否使用机器人可视化
    ifUseMsgDisp: bool = False          #是否打印机器人信息

@dataclass
class RobotStructure:
    #python数据包装类，用于机器人机械臂模型包装
    lArm_frame_names: List[str]     #左臂连杆名称列表
    rArm_frame_names: List[str]     #右臂连杆名称列表

    lArm_JntRotation_axis: List[str]    #各个关节旋转轴
    rArm_JntRotation_axis: List[str]    #各个关节旋转轴

    lArm_jointNum: int              #左臂关节数目
    rArm_jointNum: int              #右臂关节数目

@dataclass
class RobotJointMovement:
    #python数据包装类，用于机器人关节运动包装
    Joint_pos: List[float]          #关节角度
    Joint_vel: List[float]          #关节角速度
    Joint_acc: List[float]          #关节角加速度

@dataclass
class RobotBaseMovement:
    #python数据包装类，描述机器人双臂基座的运动 -- 六维度，依次为x, y, z, roll, pitch, yaw
    base_pos: List[float]       #基座笛卡尔空间位置
    base_vel: List[float]       #基座笛卡尔空间速度
    base_acc: List[float]       #基座笛卡尔空间加速度

@dataclass
class RobotAllLinkMovement:
    #python数据包装类，描述机器人双臂运动
    lArm_vel: np.array          #线速度
    lArm_omega: np.array        #角速度
    lArm_acc: np.array          #线加速度
    lArm_dOmega: np.array       #角加速度
    lArm_acc_center: np.array   #质心加速度

    rArm_vel: np.array          #线速度
    rArm_omega: np.array        #角速度
    rArm_acc: np.array          #线加速度
    rArm_dOmega: np.array       #角加速度
    rArm_acc_center: np.array   #质心加速度

#numpy矩阵打印简易代码
def print_matrix(matrix):
    """
    numpy矩阵打印
    """
    print("矩阵信息:")
    for row in matrix:
        print("[", end=" ")
        for val in row:
            print(f"{val:.4f}", end="\t")  # 保留 4 位小数
        print("]")

#机器人模型关节信息打印简易代码
def printJointInfo(model):
    """打印模型关节信息"""
    print("====================")
    print(f"DOF num of robot model is = {model.nq}")
    print("====================")
    for name, joint in zip(model.names, model.joints):
        print(f"Joint: {name}, ID: {joint.id}")

#遍历模型，依据模型受限关节名称获取其ID
def getIDviaName(model, nameList: list[str]):
    #遍历模型，依据模型受限关节名称获取其ID
    JntID = []
    for JntName in nameList:
        for name, joint in zip(model.names, model.joints):
            if JntName == name:
                JntID.append(joint.id)

    return JntID

#辅助函数：
def getRobotModel(urdf_path: str):
    robot_model = pin.RobotWrapper.BuildFromURDF(urdf_path)
    return robot_model

#辅助函数：SE3对象化作numpy数组
def pinoSE3ToNp(M):
    R = np.array(M.rotation)
    t = np.array(M.translation).reshape(3, 1)
    homogeneous_matrix = np.vstack([
        np.hstack([R, t]),
        np.array([0, 0, 0, 1])
    ])
    return homogeneous_matrix

#辅助函数：将位置矩阵转化为[x,y,z,r,p,y]六维旋量的代码
def pose_matrix_to_euler(pose_matrix):
    """
    从 4×4 位姿矩阵中提取 (x, y, z, roll, pitch, yaw)
    
    参数:
        pose_matrix: 4×4 齐次变换矩阵（numpy.ndarray）
    
    返回:
        x, y, z, roll, pitch, yaw: 位置和欧拉角（弧度）
    """
    # 提取平移部分
    x = pose_matrix[0, 3]
    y = pose_matrix[1, 3]
    z = pose_matrix[2, 3]
    
    # 提取旋转矩阵
    R = pose_matrix[:3, :3]
    
    # 计算 pitch
    pitch = np.arctan2(-R[2, 0], np.sqrt(R[0, 0]**2 + R[1, 0]**2))
    
    # 处理万向节锁（pitch 接近 ±90 度）
    if np.isclose(np.abs(pitch), np.pi / 2):
        roll = 0
        yaw = np.arctan2(R[0, 1], R[1, 1])
    else:
        # 计算 roll
        roll = np.arctan2(R[2, 1], R[2, 2])
        # 计算 yaw
        yaw = np.arctan2(R[1, 0], R[0, 0])
    
    return x, y, z, roll, pitch, yaw

#辅助函数：运算符号L定义
def L_func(vec):
    L_vec = np.array([
        [0.0, -1*vec[2], vec[1]],
        [vec[2], 0.0, -1*vec[0]],
        [-1*vec[1], vec[0], 0.0]
    ])
    return L_vec

#辅助函数：运算符号K定义
def K_func(vec):
    K_vec = np.array([
        [vec[0], vec[1], vec[2], 0.0, 0.0, 0.0],
        [0.0, vec[0], 0.0, vec[1], vec[2], 0.0],
        [0.0, 0.0, vec[0], 0.0, vec[1], vec[2]]
    ])
    return K_vec

class DualArmIK:

    #初始化函数
    def __init__(self,
                 modelWrap: RobotModelDataWrap,       #机器人模型包裹
                 model_q_st,        #机器人关节初始角度(numpy数组)
                 desc_zeroPos,      #机器人笛卡尔空间初始位姿(左臂6+右臂6: [x,y,z,roll,pitch,yaw])
                 m_limMsgWrap: RobotModelLim,
                 m_optiInte: OptiInte,          #机器人控制参数
                 m_viewInte: ViewMsg            #机器人信息展示的参数
                 ):

        #装入原始的机器人模型
        self.origin_robot = pin.RobotWrapper.BuildFromURDF(modelWrap.model_path)    #创建模型
        self.originRobot_DOF = self.origin_robot.model.nq     #原始模型自由度数量
        self.lArm_EndLabel = modelWrap.ArmEndLabel[0]
        self.rArm_EndLabel = modelWrap.ArmEndLabel[1]
        if m_limMsgWrap.ifDispJntMsg:
            print("限位前关节：")
            printJointInfo(self.origin_robot.model)

        #提取机器人各级关节子连杆的惯性参数（按关节ID索引的list，跳过universe根帧）
        self.link_mass = []           # 子连杆质量
        self.link_inertia = []        # 子连杆3x3惯量矩阵（含惯性矩和惯性积）
        self.link_com_local = []      # 子连杆质心在关节坐标系下的位置

        for i in range(self.origin_robot.model.njoints):
            if i == 0:      # 跳过 universe 根帧
                continue
            inertia_i = self.origin_robot.model.inertias[i]
            self.link_mass.append(inertia_i.mass if inertia_i.mass is not None else 0.0)
            self.link_inertia.append(np.array(inertia_i.inertia) if inertia_i.inertia is not None else np.zeros((3, 3)))
            self.link_com_local.append(np.array(inertia_i.lever) if inertia_i.lever is not None else np.zeros(3))

        #针对机器人角度进行限制
        self.jntRef = np.array(model_q_st)                      #角度参考值

        #笛卡尔空间初始值（默认/归零目标位姿，来源：desc_zeropos.yaml）
        self.lArm_descZeroPos = np.array(desc_zeroPos[0:6])     #左臂末端零位 [x,y,z,roll,pitch,yaw]
        self.rArm_descZeroPos = np.array(desc_zeroPos[6:12])    #右臂末端零位 [x,y,z,roll,pitch,yaw]

        self.ifUseLim = m_limMsgWrap.ifUseLim                   #判断量：是否开启关节限制
        self.limJntLabel = m_limMsgWrap.LimJntLabelList         #受限关节标签
        self.limJntID = getIDviaName(self.origin_robot.model, self.limJntLabel)     #受限关节ID
        self.LimJntAng = []             #受限关节角度
        for tempID in self.limJntID:
            self.LimJntAng.append(model_q_st[tempID - 1])

        self.ifLstUseLim = m_limMsgWrap.ifUseLim             #辅助变量 —— 判断上一轮初始化中是否加了限制，用于后续更新过程
        self.LastLimJntLabel = m_limMsgWrap.LimJntLabelList     #辅助变量 —— 用于存储上一轮限制中限制了哪几个关节
        self.LastLimJntID = getIDviaName(self.origin_robot.model, self.LastLimJntLabel)     #辅助变量 —— 用于储存上一轮的限制中限制关节的ID
        self.LastLimJntAng = []             #辅助变量 —— 用于储存上一轮限制中限制的关节的角度
        for tempID in self.LastLimJntID:
            self.LastLimJntAng.append(model_q_st[tempID - 1])

        #创建受限的机器人模型 -- 实际求解中用受限机器人模型求解
        if self.ifUseLim:

            #创建受限机器人模型
            self.reduced_robot = self.origin_robot.buildReducedRobot(
                self.limJntLabel,
                self.jntRef,
            )
            self.reduced_robot_DOF = self.reduced_robot.model.nq

            #获取当前机器人模型所有关节角度，作为初始角度
            self.q_st = np.array(self.reduced_robot.q0)

            #打印信息
            if m_limMsgWrap.ifDispJntMsg:
                print("限位后机器人关节信息：")
                printJointInfo(self.reduced_robot.model)
                print("机器人关节初始角度：")
                print(self.q_st)
        else:
            self.reduced_robot = self.origin_robot
            self.reduced_robot_DOF = self.reduced_robot.model.nq

            #获取当前机器人模型所有关节角度，作为初始角度
            self.q_st = np.array(self.reduced_robot.q0)

            if m_limMsgWrap.ifDispJntMsg:
                print("未使用关节限位")
                print("机器人关节初始角度：")
                print(self.q_st)

        #开关量：
        self.Visualization = m_viewInte.ifVisu      #开关量——是否使用可视化
        self.ifUseMsgDisp = m_viewInte.ifUseMsgDisp        #开关量——是否打印机器人信息

        #打印机器人信息 -- 关节名称、关节ID、关节类型、关节限位（若有）
        if self.ifUseMsgDisp:
            print("\n===== Robot Model Information =====")
            print(f"Model Name: {self.reduced_robot.model.name}")
            print(f"Degrees of Freedom (DOF): {self.reduced_robot.model.nq}")
            
            # 打印关节信息
            print("\nJoints Information:")
            for i in range(self.reduced_robot.model.njoints):
                joint_name = self.reduced_robot.model.names[i]
                joint_type = "Revolute" if self.reduced_robot.model.joints[i].nq == 1 else "Fixed"
                print(f"  ID: {i}, Name: {joint_name}, Type: {joint_type}")
            
            # 打印末端执行器信息（仅使用输入参数 ArmEndLabel，避免访问未初始化的属性）
            print("\nEnd Effectors (Input Labels):")
            print(f"  Left Arm: {self.lArm_EndLabel}")
            print(f"  Right Arm: {self.rArm_EndLabel}")
            
            # 打印关节限位（如果有）
            print("\nJoint Position Limits:")
            lower_limits = self.reduced_robot.model.lowerPositionLimit
            upper_limits = self.reduced_robot.model.upperPositionLimit
            for i in range(self.reduced_robot.model.nq):
                print(f"  Joint {i}: Lower={lower_limits[i]:.4f}, Upper={upper_limits[i]:.4f}")

             # 添加打印帧信息的代码
            print("\nFrame Information:")
            print(f"Total number of frames: {len(self.reduced_robot.model.frames)}")
            print("\nJOINT and BODY frames:")
            for frame_id, frame in enumerate(self.reduced_robot.model.frames):
                if frame.type == pin.FrameType.JOINT or frame.type == pin.FrameType.BODY:
                    frame_type = "JOINT" if frame.type == pin.FrameType.JOINT else "BODY"
                    parent_joint_name = self.reduced_robot.model.names[frame.parent]
                    print(f"  ID: {frame_id}, Type: {frame_type}, Name: {frame.name}, Parent Joint: {parent_joint_name}")

            print("===================================\n")

        #建立casadi逆解求解器模型
        self.cmodel = cpin.Model(self.reduced_robot.model)
        self.cdata = self.cmodel.createData()

        #casadi逆解求解参数确定
        self.cq = casadi.SX.sym("q", self.reduced_robot.model.nq, 1) 
        self.cTf_l = casadi.SX.sym("tf_l", 4, 4)
        self.cTf_r = casadi.SX.sym("tf_r", 4, 4)
        cpin.framesForwardKinematics(self.cmodel, self.cdata, self.cq)

        #获取末端ID并定义误差函数
        self.base_frame_id = m_optiInte.baseFrameID
        self.L_hand_id = self.reduced_robot.model.getFrameId(self.lArm_EndLabel)
        self.R_hand_id = self.reduced_robot.model.getFrameId(self.rArm_EndLabel)

        self.translational_error = casadi.Function(
            "translational_error",
            [self.cq, self.cTf_l, self.cTf_r],
            [
                casadi.vertcat(
                    self.cdata.oMf[self.L_hand_id].translation - (self.cdata.oMf[self.base_frame_id].homogeneous@self.cTf_l)[:3,3],
                    self.cdata.oMf[self.R_hand_id].translation - (self.cdata.oMf[self.base_frame_id].homogeneous@self.cTf_r)[:3,3]
                )
            ]
        )
        self.rotational_error = casadi.Function(
            "rotational_error",
            [self.cq, self.cTf_l, self.cTf_r],
            [
                casadi.vertcat(
                    cpin.log3(self.cdata.oMf[self.L_hand_id].rotation @ (self.cdata.oMf[self.base_frame_id].homogeneous@self.cTf_l)[:3,:3].T),
                    cpin.log3(self.cdata.oMf[self.R_hand_id].rotation @ (self.cdata.oMf[self.base_frame_id].homogeneous@self.cTf_r)[:3,:3].T)
                )
            ]
        )

        #优化问题：
        self.tempTime = m_optiInte.tempTime

        self.opti = casadi.Opti()       #初始化优化问题
        self.var_q = self.opti.variable(self.reduced_robot.model.nq)    #定义优化变量：机器人关节配置

        #定义参数(固定输入)
        self.var_q_last = self.opti.parameter(self.reduced_robot.model.nq)      #平滑项辅助参数 -- 上一次关节位置
        self.var_v_last = self.opti.parameter(self.reduced_robot.model.nq)      #平滑项辅助参数 -- 上一次关节速度
        self.param_tf_l = self.opti.parameter(4, 4)     #左臂目标位置(齐次变换矩阵)
        self.param_tf_r = self.opti.parameter(4, 4)     #右臂目标位置(齐次变换矩阵)
        #定义目标函数(成本函数)：
        self.translational_cost = casadi.sumsqr(self.translational_error(self.var_q, self.param_tf_l, self.param_tf_r))     #位移误差成本
        self.rotation_cost = casadi.sumsqr(self.rotational_error(self.var_q, self.param_tf_l, self.param_tf_r))     #旋转误差成本
        self.regularization_cost = casadi.sumsqr(self.var_q)            #正则化成本(避免关节角度过大)
        self.smooth_cost = casadi.sumsqr(self.var_q - self.var_q_last)      #平滑成本(最小化当前关节配置与上一时刻配置差值，使运动平滑)
        self.vel_smooth_cost = casadi.sumsqr((self.var_q - self.var_q_last)/self.tempTime - self.var_v_last)    #平滑成本（最小化当前关节速度与上一刻差值）


        #优化问题约束(关节位置限制)
        self.opti.subject_to(
            self.opti.bounded(
                self.reduced_robot.model.lowerPositionLimit,
                self.var_q,
                self.reduced_robot.model.upperPositionLimit
            )
        )

        #优化目标函数(成本函数的加权组合)
        self.Weight_param = self.opti.parameter(5)
        self.opti.minimize(
            self.Weight_param[0] * self.translational_cost 
            + self.Weight_param[1] * self.rotation_cost 
            + self.Weight_param[2] * self.regularization_cost 
            + self.Weight_param[3] * self.smooth_cost
            + self.Weight_param[4] * self.vel_smooth_cost
        )
        # 储存初始化权重
        self.optiWeight_origin = m_optiInte.optiWeight
        self.optiWeight_store = m_optiInte.optiWeight
        # 权重注入
        self.opti.set_value(self.Weight_param , m_optiInte.optiWeight)

        #求解器配置
        opts = {
            'ipopt': {
                'print_level': m_optiInte.print_level,       # 不打印优化日志
                'max_iter': m_optiInte.max_iter,         # 最大迭代次数
                'tol': m_optiInte.tol             # 收敛容忍度
            },
            'print_time': m_optiInte.print_time,        # 不打印计算时间
            'calc_lam_p': False         # 避免 NaN 问题（CasADi FAQ）
        }
        self.opts = opts
        self.opti.solver("ipopt", opts)

        #初始化数据
        q_st_np = np.array(self.q_st)
        if len(q_st_np) == self.reduced_robot.model.nq:
            #self.init_data = np.zeros(self.reduced_robot.model.nq)  # 初始关节配置（全零）
            self.init_data = q_st_np    #初始化关节配置（按照初始位置初始化）

            self.init_vel = np.zeros(len(self.init_data))
            self.last_init_data = self.init_data.copy()    # 保存上一次位置用于速度计算
        else:
            print(f"Error(IK-01): incorrect init input -- joint ang array len incorrect")

        #设置可视化代码
        if self.Visualization:
            # Initialize the Meshcat visualizer for visualization
            self.vis = MeshcatVisualizer(self.origin_robot.model, self.origin_robot.collision_model, self.origin_robot.visual_model)
            self.vis.initViewer(open=True) 
            self.vis.loadViewerModel("pinocchio") 
            
            # 如果开关量为 True，则显示所有关节的坐标系
            self.vis.displayFrames(True, frame_ids=[self.base_frame_id], axis_length=0.2, axis_width=5)
            self.vis.displayFrames(True, frame_ids=[self.L_hand_id, self.R_hand_id], axis_length = 0.15, axis_width = 5)
            
            self.vis.display(pin.neutral(self.origin_robot.model))

            # Enable the display of end effector target frames with short axis lengths and greater width.
            frame_viz_names = [self.lArm_EndLabel, self.rArm_EndLabel]
            FRAME_AXIS_POSITIONS = (
                np.array([[0, 0, 0], [1, 0, 0],
                          [0, 0, 0], [0, 1, 0],
                          [0, 0, 0], [0, 0, 1]]).astype(np.float32).T
            )
            FRAME_AXIS_COLORS = (
                np.array([[1, 0, 0], [1, 0.6, 0],
                          [0, 1, 0], [0.6, 1, 0],
                          [0, 0, 1], [0, 0.6, 1]]).astype(np.float32).T
            )
            axis_length = 0.1
            axis_width = 20
            for frame_viz_name in frame_viz_names:
                self.vis.viewer[frame_viz_name].set_object(
                    mg.LineSegments(
                        mg.PointsGeometry(
                            position=axis_length * FRAME_AXIS_POSITIONS,
                            color=FRAME_AXIS_COLORS,
                        ),
                        mg.LineBasicMaterial(
                            linewidth=axis_width,
                            vertexColors=True,
                        ),
                    )
                )


#===========================================================================================================================
#辅助函数

    #辅助函数：将输入目标位姿基坐标系转换的函数
    def desPos_baseFrameTrans(self, desPos_origin):
        #原始基坐标系 -- arm_base_link
        #转换后基坐标系 -- universe

        # 获取符号旋转矩阵
        R_sym = self.cdata.oMf[self.base_frame_id].rotation  # 3x3 SX
        t_sym = self.cdata.oMf[self.base_frame_id].translation  # 3x1 SX

        # 转换为 NumPy（需要先求值）
        R_np = np.array([[float(R_sym[i, j]) for j in range(3)] for i in range(3)])
        t_np = np.array([float(t_sym[i]) for i in range(3)])

        # 构造 4x4 位姿矩阵
        left_wrist_trans_np = np.eye(4)
        left_wrist_trans_np[:3, :3] = R_np
        left_wrist_trans_np[:3, 3] = t_np

        desPos_Transed = left_wrist_trans_np @ desPos_origin
        return desPos_Transed
    
    #辅助函数：适用于numpy数组的转化方法，将目标位姿由arm_base_link为基坐标系转化到universe为基坐标系
    def desPos_baseFrameTrans_np(self, desPos_origin):
        #适用于numpy数组的转化方法
        # 获取符号旋转矩阵
        R_sym = self.cdata.oMf[self.base_frame_id].rotation  # 3x3 SX
        t_sym = self.cdata.oMf[self.base_frame_id].translation  # 3x1 SX

        # 转换为 NumPy（需要先求值）
        R_np = np.array([[float(R_sym[i, j]) for j in range(3)] for i in range(3)])
        t_np = np.array([float(t_sym[i]) for i in range(3)])

        # 构造齐次变换矩阵（4x4）
        base_to_universe = np.eye(4)
        base_to_universe[:3, :3] = R_np
        base_to_universe[:3, 3] = t_np.flatten()  # 确保是 1D 数组

        # 处理输入位姿（确保是齐次矩阵）
        if desPos_origin.shape == (3, 4):
            desPos_origin_h = np.vstack([desPos_origin, [0, 0, 0, 1]])
        elif desPos_origin.shape == (4, 4):
            desPos_origin_h = desPos_origin
        else:
            raise ValueError("输入位姿必须是 3x4 或 4x4 的 NumPy 数组")

        # 坐标变换：T_universe = T_base_to_universe * T_origin
        desPos_Transed = base_to_universe @ desPos_origin_h
        return desPos_Transed
    
    #辅助函数：适用于numpy数组的转化方法，将输入位姿由universe转化到arm_base_link
    def desPos_baseFrameTrans_inverse(self, desPos_universe):
        """
        将输入位姿从universe坐标系转换到arm_base_link坐标系
        
        参数:
            desPos_universe: 在universe坐标系中的位姿 (3x4或4x4 numpy数组)
            
        返回:
            desPos_base: 在arm_base_link坐标系中的位姿 (4x4 numpy数组)
        """
        # 转换后基坐标系 -- arm_base_link
        # 原始基坐标系 -- universe

        # 获取符号旋转矩阵和平移向量
        R_sym = self.cdata.oMf[self.base_frame_id].rotation  # 3x3 SX
        t_sym = self.cdata.oMf[self.base_frame_id].translation  # 3x1 SX

        # 转换为 NumPy（需要先求值）
        R_np = np.array([[float(R_sym[i, j]) for j in range(3)] for i in range(3)])
        t_np = np.array([float(t_sym[i]) for i in range(3)])

        # 构造从universe到arm_base_link的齐次变换矩阵
        # T_universe_to_base = T_base_to_universe^(-1)
        universe_to_base = np.eye(4)
        universe_to_base[:3, :3] = R_np.T  # 旋转矩阵的逆等于转置
        universe_to_base[:3, 3] = -R_np.T @ t_np  # 平移部分：-R^T * t

        # 处理输入位姿（确保是齐次矩阵）
        if desPos_universe.shape == (3, 4):
            desPos_universe_h = np.vstack([desPos_universe, [0, 0, 0, 1]])
        elif desPos_universe.shape == (4, 4):
            desPos_universe_h = desPos_universe
        else:
            raise ValueError("输入位姿必须是 3x4 或 4x4 的 NumPy 数组")

        # 坐标变换：T_base = T_universe_to_base * T_universe
        desPos_base = universe_to_base @ desPos_universe_h
        
        return desPos_base

    #辅助函数：类型转换 -- 将SE3类型数据转化成六维旋量
    def transSE3_to_vec(self, SE3_matrix):
        pos = SE3_matrix.translation
        rot = Rotation.from_matrix(SE3_matrix.rotation).as_euler('xyz', degrees=False)

        vecRes = np.hstack([pos, rot])
        return vecRes
    
    #辅助函数：类型转换 -- 将numpy思维数组转化成六维旋量
    def transNpMat_to_vec(self, np_Mat):
        pos = np_Mat[:3, 3]
        rot = Rotation.from_matrix(np_Mat[:3, :3]).as_euler('xyz', degrees=False)

        vecRes = np.hstack([pos, rot])
        return vecRes

#===========================================================================================================================
#功能函数

    #辅助函数：可视化信息发送与更新函数
    def qResCompleteFunc(self, q_res):
        #输入量——角度求解结果
        #该函数自动识别模型是否为削减自由度的模型，若是，则依据角度限制将自由度数目补齐，并将数据发送给可视化模块
        q_res_send = np.array(q_res)        #接受并转化为numpy数组再进行操作

        if self.ifUseLim:
            #存在限制，予以补全
            for JntID, JntAng in zip(self.limJntID, self.LimJntAng):
                q_res_send = np.insert(q_res_send, JntID-1, JntAng)
        return q_res_send

#===========================================================================================================================
    #求解模式更新函数
    def optiMod_update(self, optiWeight_mode, optiWeight_Inp, ifUseFtr, ifRefresh_pos):
        """
        求解模式定义函数 -- 定义权重以及是否使用滤波器
        ----------------------
        输入量：
            optiWeight_mode -- 权重输入模式：1 -- 经初始化的默认权重；2 -- 自定义权重
            optiWeight_Inp -- 自定义权重系数
            ifUseFtr -- 是否使用滤波器
            ifRefresh_pos -- 是否刷新求解器上一轮位置为初始位置（适配于点位控制）
        """

        if ifRefresh_pos:
            #将机器人参数刷新到零位
            self.init_data = self.q_st.copy()
            self.init_vel = np.zeros(len(self.init_data))
            self.last_init_data = self.init_data.copy()
            self.opti.set_initial(self.var_q, self.init_data)
            self.opti.set_value(self.var_q_last, self.init_data)
            self.opti.set_value(self.var_v_last, self.init_vel)

        #刷新权重
        if optiWeight_mode == 1:
            self.opti.set_value(self.Weight_param, self.optiWeight_origin)
            self.optiWeight_store = self.optiWeight_origin
        else:
            self.opti.set_value(self.Weight_param, optiWeight_Inp)
            self.optiWeight_store = optiWeight_Inp

    #逆解求解函数
    def solve_ik(self, left_wrist, right_wrist,             #目标位姿(4X4齐次变换矩阵)
                 current_lr_arm_motor_q = None,             #当前关节角度
                 ):
        #初始化优化器，若提供当前关节角度，用其初始优化器，否则用默认值——0
        if current_lr_arm_motor_q is not None:
            current_q_full = np.array(current_lr_arm_motor_q)

            #更新速度计算
            if hasattr(self, 'last_init_data'):
                # 计算当前速度 = (当前位置 - 上一次位置) / 时间步长
                current_q_reduced = current_q_full.copy()
                if self.ifUseLim:
                    indices_to_remove = np.array([JntID - 1 for JntID in self.limJntID])
                    current_q_reduced = np.delete(current_q_reduced, indices_to_remove)
                
                # 检查维度匹配
                if len(current_q_reduced) == len(self.last_init_data):
                    self.init_vel = (current_q_reduced - self.last_init_data) / self.tempTime
                    self.last_init_data = self.init_data.copy()  # 保存上一次数据
                    self.init_data = current_q_reduced.copy()    # 更新当前位置
            else:
                # 第一次调用，使用提供的关节角度
                if self.ifUseLim:
                    indices_to_remove = np.array([JntID - 1 for JntID in self.limJntID])
                    self.init_data = np.delete(current_q_full, indices_to_remove)
                else:
                    self.init_data = current_q_full.copy()
                self.init_vel = np.zeros(len(self.init_data))  # 初始速度为零
                self.last_init_data = self.init_data.copy()    # 初始化上一次数据

        if self.Visualization:
            print(self.init_data)
        self.opti.set_initial(self.var_q, self.init_data)

        #设置优化问题参数
        self.opti.set_value(self.param_tf_l, left_wrist)  # 设置左腕目标位姿
        self.opti.set_value(self.param_tf_r, right_wrist) # 设置右腕目标位姿
        self.opti.set_value(self.var_q_last, self.init_data)  # 用于平滑过渡（避免突变）
        self.opti.set_value(self.var_v_last, self.init_vel)     # 设置上一次关节速度

        #设置可视化参数
        if self.Visualization:
            left_wrist_trans = self.desPos_baseFrameTrans(left_wrist)
            right_wrist_trans = self.desPos_baseFrameTrans(right_wrist)
            self.vis.viewer[self.lArm_EndLabel].set_transform(left_wrist_trans)   # 显示左腕目标位姿
            self.vis.viewer[self.rArm_EndLabel].set_transform(right_wrist_trans)  # 显示右腕目标位姿

        #求解优化问题
        try:
            sol = self.opti.solve()  # 尝试求解
            sol_q = self.opti.value(self.var_q)  # 提取解（关节角度）

            # 更新速度和位置用于下一次迭代
            self.init_vel = (sol_q - self.init_data) / self.tempTime  # 计算新速度
            self.last_init_data = self.init_data.copy()  # 保存当前数据作为下一次的"上一次数据"
            self.init_data = sol_q.copy()  # 更新为新的解

            sol_q_whole = self.qResCompleteFunc(sol_q)

            if self.Visualization:
                # 显示求解后的机器人关节角度
                self.vis.display(sol_q_whole)


        except:
            print(f"Error(IK-02): ERROR in IK solving -- No result found")
            sol_q = self.init_data.copy()
            self.last_init_data = sol_q.copy()
            self.init_vel = np.zeros(len(sol_q))
            sol_q_whole = self.qResCompleteFunc(sol_q)

        #返回求解结果
        return sol_q_whole
    
    #逆解求解函数封装 -- 返回lists类型的数据
    def solveIk_ListRet(self, left_wrist, right_wrist,      #目标位姿(4X4齐次变换)
                        current_lr_arm_motor_q = None,
                        ):
        sol_q = self.solve_ik(left_wrist, right_wrist, current_lr_arm_motor_q)

        #sol_q_np = sol_q.full()
        sol_q_list = sol_q.tolist()
        return sol_q_list

    #正解求解函数：
    def solveFk(self, arm_motor_q):
        """
        计算机器人双臂在给定关节角度下的末端执行器位姿（正运动学）
        
        参数:
            arm_motor_q: 机器人关节角度配置 (numpy数组，形状为 [nq,] 或 [1, nq])
            
        返回:
            left_wrist_pose: 左臂末端执行器位姿 (4x4齐次变换矩阵，pinocchio.SE3格式)
            right_wrist_pose: 右臂末端执行器位姿 (4x4齐次变换矩阵，pinocchio.SE3格式)
            注意这两个返回值对应的基坐标系为 arm_base_link 坐标系
        """
        # 确保输入是numpy数组
        q = np.array(arm_motor_q).flatten()

        # 依据reduced_robot情况削减角度
        if self.ifUseLim:
            indeices_to_remove = np.array([JntID - 1 for JntID in self.limJntID])
            q = np.delete(q, indeices_to_remove)        #删除对应索引的数据

        # 检查输入维度是否正确
        if q.shape[0] != self.reduced_robot.model.nq:
            raise ValueError(f"输入关节角度维度不正确，应为 {self.reduced_robot.model.nq} 维，实际上为：{len(q)} 维")
        
        # 创建数据对象并更新关节配置
        data = self.reduced_robot.model.createData()
        pin.forwardKinematics(self.reduced_robot.model, data, q)
        pin.updateFramePlacements(self.reduced_robot.model, data)

        left_ee_id = self.reduced_robot.model.getFrameId(self.lArm_EndLabel)
        right_ee_id = self.reduced_robot.model.getFrameId(self.rArm_EndLabel)
        left_ee_pos = data.oMf[left_ee_id]
        right_ee_pos = data.oMf[right_ee_id]

        # 变换到 arm_base_link 坐标系，与 solveFk_ListRet 保持一致
        base_pos = data.oMf[self.base_frame_id]
        left_ee_pos = base_pos.inverse() * left_ee_pos
        right_ee_pos = base_pos.inverse() * right_ee_pos

        return left_ee_pos, right_ee_pos
    
    #正解求解函数封装 -- 返回Lists类型的数据
    def solveFk_ListRet(self, arm_motor_q):
        #输入量——关节角度的列表(lists)

        #求解pinocchio::SE3类型的返回值（已变换到 arm_base_link 坐标系）
        left_ee_pos, right_ee_pos = self.solveFk(arm_motor_q)

        #返回值转换为 numpy 矩阵
        left_ee_pos_np = pinoSE3ToNp(left_ee_pos)
        right_ee_pos_np = pinoSE3ToNp(right_ee_pos)

        left_ee_pos_list = []
        right_ee_pos_list = []

        # 左臂末端位姿转换（4x4齐次变换矩阵展开为16个元素的列表，按行优先）
        for i in range(4):  # 行
            for j in range(4):  # 列
                left_ee_pos_list.append(left_ee_pos_np[i, j])

        # 右臂末端位姿转换（4x4齐次变换矩阵展开为16个元素的列表，按行优先）
        for i in range(4):  # 行
            for j in range(4):  # 列
                right_ee_pos_list.append(right_ee_pos_np[i, j])

        # 返回左右臂位姿的列表（每个列表包含16个元素，表示4x4矩阵）
        return left_ee_pos_list, right_ee_pos_list

    #修改限位接口
    def JntLimReDef(self, LimMsg: RobotModelLim):
        #重新设定关节位置限制与初始化方法

        #获取受限信息：
        self.jntRef = np.array(self.origin_robot.q0)        # 始终使用原始完整模型的参考位形
        self.ifUseLim = LimMsg.ifUseLim
        self.limJntLabel = LimMsg.LimJntLabelList
        self.limJntID = getIDviaName(self.origin_robot.model, self.limJntLabel)
        self.LimJntAng = LimMsg.LimJntAngList

        #################################################debug
        #print("redef success")
        #print(f"ref joint unprocessed = {self.jntRef}")

        #将限制的关节角度添加到jntRef里面去
        all_limJntID = getIDviaName(self.origin_robot.model, self.limJntLabel)
        for JntAng, JntID in zip(LimMsg.LimJntAngList, all_limJntID):
            if self.ifLstUseLim:
                self.jntRef = np.insert(self.jntRef, JntID-1, JntAng)
            else:
                self.jntRef[JntID - 1] = JntAng
        if LimMsg.ifDispJntMsg:
            print("初始位置为：")
            print(self.jntRef)

        #print(f"ref joint limited = {self.jntRef}")

        #对init_data进行处理，防止不匹配
        if self.ifLstUseLim == self.ifUseLim:
            #两轮均采用了限制，但是限制的关节对象可能不同，因此需要判断一下，不同情况不同处理
            if self.LastLimJntLabel == self.limJntLabel:
                #两次限制相同          
                self.init_data = self.init_data
            else:
                #两次限制不同
                #先填补上一轮的角度限制值
                for JntAng, JntID in zip(self.LastLimJntAng, self.LastLimJntID):
                    self.init_data = np.insert(self.init_data, JntID - 1, JntAng)
                #再去掉这一轮的角度限制值
                indices_to_remove = np.array([JntID - 1 for JntID in self.limJntID])  # 计算要删除的索引
                self.init_data = np.delete(self.init_data, indices_to_remove)  # 删除对应索引的数据
                
        elif self.ifLstUseLim and not self.ifUseLim:
            #上一轮采用了限制而这一轮没有限制，此时需要将上一轮扣去的角度添加回来
            for JntAng, JntID in zip(self.LastLimJntAng, self.LastLimJntID):
                self.init_data = np.insert(self.init_data, JntID - 1, JntAng)
        elif not self.ifLstUseLim and self.ifUseLim:
            #上一轮未采用限制，而这一轮采用了限制
            indices_to_remove = np.array([JntID - 1 for JntID in self.limJntID])  # 计算要删除的索引
            self.init_data = np.delete(self.init_data, indices_to_remove)  # 删除对应索引的数据

        #创建模型
        if self.ifUseLim:
            #创建简化的模型
            self.reduced_robot = self.origin_robot.buildReducedRobot(
                self.limJntLabel,
                self.jntRef
            )
            self.reduced_robot_DOF = self.reduced_robot.model.nq

            if LimMsg.ifDispJntMsg:
                print("限位后机器人关节信息：")
                printJointInfo(self.reduced_robot.model)
        else:
            #使用原始的模型
            self.reduced_robot = self.origin_robot
            self.reduced_robot_DOF = self.reduced_robot.model.nq

            if LimMsg.ifDispJntMsg:
                print("未使用关节限位")

        #重新查询末端帧ID（模型重建后索引可能位移）
        self.L_hand_id = self.reduced_robot.model.getFrameId(self.lArm_EndLabel)
        self.R_hand_id = self.reduced_robot.model.getFrameId(self.rArm_EndLabel)

        #依据新的模型更新优化问题
        #刷新casadi逆解求解器模型
        self.cmodel = cpin.Model(self.reduced_robot.model)
        self.cdata = self.cmodel.createData()
        self.cq = casadi.SX.sym("q", self.reduced_robot.model.nq, 1)
        cpin.framesForwardKinematics(self.cmodel, self.cdata, self.cq)

        #获取末端ID并定义误差函数
        self.translational_error = casadi.Function(
            "translational_error",
            [self.cq, self.cTf_l, self.cTf_r],
            [
                casadi.vertcat(
                    self.cdata.oMf[self.L_hand_id].translation - (self.cdata.oMf[self.base_frame_id].homogeneous@self.cTf_l)[:3,3],
                    self.cdata.oMf[self.R_hand_id].translation - (self.cdata.oMf[self.base_frame_id].homogeneous@self.cTf_r)[:3,3]
                )
            ]
        )
        self.rotational_error = casadi.Function(
            "rotational_error",
            [self.cq, self.cTf_l, self.cTf_r],
            [
                casadi.vertcat(
                    cpin.log3(self.cdata.oMf[self.L_hand_id].rotation @ (self.cdata.oMf[self.base_frame_id].homogeneous@self.cTf_l)[:3,:3].T),
                    cpin.log3(self.cdata.oMf[self.R_hand_id].rotation @ (self.cdata.oMf[self.base_frame_id].homogeneous@self.cTf_r)[:3,:3].T)
                )
            ]
        )

        #优化问题更新：
        self.opti = casadi.Opti()       #更新优化问题
        self.var_q = self.opti.variable(self.reduced_robot.model.nq)    #定义优化变量：机器人关节配置

        #定义参数(固定输入)
        self.var_q_last = self.opti.parameter(self.reduced_robot.model.nq)      #平滑项辅助参数 -- 上一轮关节的位置
        self.var_v_last = self.opti.parameter(self.reduced_robot.model.nq)      # 上一次的关节速度
        self.Weight_param = self.opti.parameter(5)                              # 权重参数（可运行时修改）

        self.param_tf_l = self.opti.parameter(4, 4)     #左臂目标位置(齐次变换矩阵)
        self.param_tf_r = self.opti.parameter(4, 4)     #右臂目标位置(齐次变换矩阵)

        #定义目标函数(成本函数)：
        self.translational_cost = casadi.sumsqr(self.translational_error(self.var_q, self.param_tf_l, self.param_tf_r))     #位移误差成本
        self.rotation_cost = casadi.sumsqr(self.rotational_error(self.var_q, self.param_tf_l, self.param_tf_r))     #旋转误差成本
        self.regularization_cost = casadi.sumsqr(self.var_q)            #正则化成本(避免关节角度过大)
        self.smooth_cost = casadi.sumsqr(self.var_q - self.var_q_last)      #平滑成本(最小化当前关节配置与上一时刻配置差值，使运动平滑)
        # 添加速度平滑成本
        self.vel_smooth_cost = casadi.sumsqr(
            (self.var_q - self.var_q_last) / self.tempTime - self.var_v_last
        )

        #优化问题约束(关节位置限制)
        self.opti.subject_to(
            self.opti.bounded(
                self.reduced_robot.model.lowerPositionLimit,
                self.var_q,
                self.reduced_robot.model.upperPositionLimit
            )
        )

        #优化目标函数(成本函数的加权组合)
        self.opti.minimize(
            self.Weight_param[0] * self.translational_cost
            + self.Weight_param[1] * self.rotation_cost
            + self.Weight_param[2] * self.regularization_cost
            + self.Weight_param[3] * self.smooth_cost
            + self.Weight_param[4] * self.vel_smooth_cost
        )
        # 注入当前权重值，保持与store一致
        self.opti.set_value(self.Weight_param, self.optiWeight_store)

        #求解器配置
        self.opti.solver("ipopt", self.opts)

        # 重新初始化速度和上一次位置数据
        self.init_vel = np.zeros(len(self.init_data))  # 重新初始化速度
        self.last_init_data = self.init_data.copy()    # 重新初始化上一次位置

        #更新辅助变量
        self.ifLstUseLim = LimMsg.ifUseLim
        self.LastLimJntLabel = self.limJntLabel
        self.LastLimJntID = self.limJntID
        self.LastLimJntAng = LimMsg.LimJntAngList

    #运动求解接口 -- 已知关节运动，求解机器人各级杆件运动
    def RobotDyModel_linkMovement(self, 
                              robot_arm_joint: RobotStructure,
                              arm_motor_movement:RobotJointMovement, 
                              base_Cartesian_movement:RobotBaseMovement, 
                              current_lr_arm_motor_q = None,
                            ):
    
        #将机器人现阶段角度复制到机器人模型中
        if current_lr_arm_motor_q is not None:
            q_current = np.array(current_lr_arm_motor_q).flatten()
        else:
            q_current = np.zeros(14)
        data = self.origin_robot.model.createData()
        pin.forwardKinematics(self.origin_robot.model, data, q_current)
        pin.updateFramePlacements(self.origin_robot.model, data)

        #已知机器人末端受力，求解机器人各个关节受力
        lArm_link_vel_cart = []         #左臂线速度
        lArm_link_omega_cart = []       #左臂角速度

        lArm_link_acc_cart = []         #左臂线加速度 -- 坐标系
        lArm_link_dOmega_cart = []      #左臂角加速度 -- 坐标系
        lArm_link_acc_cart_center = []  #左臂线加速度 -- 质心位置

        rArm_link_vel_cart = []         #右臂线速度
        rArm_link_omega_cart = []       #右臂角速度

        rArm_link_acc_cart = []         #右臂线加速度 -- 坐标系
        rArm_link_dOmega_cart = []      #右臂角加速度 -- 坐标系
        rArm_link_acc_cart_center = []  #右臂线加速度 -- 质心位置


        #通过pinocchio获取转换矩阵列表
        #前向第推转换矩阵 -- 从基座逐级向末端第推
        lArm_transmit_forward = []
        rArm_transmit_forward = []
        #反向第推转换矩阵 -- 从末端逐级向基座第推
        lArm_transmit_backward = []
        rArm_transmit_backward = []

        # 从origin_robot模型提取各级转换矩阵
        lArm_frame_ids = [self.origin_robot.model.getFrameId(name)
                          for name in robot_arm_joint.lArm_frame_names]
        rArm_frame_ids = [self.origin_robot.model.getFrameId(name)
                          for name in robot_arm_joint.rArm_frame_names]

        for i, fid in enumerate(lArm_frame_ids):
            if i == 0:
                # 第一级：world → link_0
                T_fwd = pinoSE3ToNp(data.oMf[fid])
            else:
                # link_{i-1} → link_i
                T_fwd = pinoSE3ToNp(
                    data.oMf[lArm_frame_ids[i-1]].inverse() * data.oMf[fid]
                )
            lArm_transmit_forward.append(T_fwd)
            lArm_transmit_backward.append(np.linalg.inv(T_fwd))

        for i, fid in enumerate(rArm_frame_ids):
            if i == 0:
                T_fwd = pinoSE3ToNp(data.oMf[fid])
            else:
                T_fwd = pinoSE3ToNp(
                    data.oMf[rArm_frame_ids[i-1]].inverse() * data.oMf[fid]
                )
            rArm_transmit_forward.append(T_fwd)
            rArm_transmit_backward.append(np.linalg.inv(T_fwd))

        # ---------- 前向递推过程 ----------
        #针对左臂前向传递第推
        for i in range(robot_arm_joint.lArm_jointNum):
            if robot_arm_joint.lArm_JntRotation_axis[i] == 'x':
                cho_vec_3D = np.array([1, 0, 0])
            elif robot_arm_joint.lArm_JntRotation_axis[i] == 'y':
                cho_vec_3D = np.array([0, 1, 0])
            else:
                cho_vec_3D = np.array([0, 0, 1])

            temp_R = lArm_transmit_backward[i][:3, :3]
            temp_P = lArm_transmit_backward[i][:3, 3]

            # ----- 针对速度处理 -----
            if i == 0:
                #针对第一级处理 -- 已知世界坐标系0相对于自身的运动
                vel_0 = np.array(base_Cartesian_movement.base_vel[0:3])
                omega_0 = np.array(base_Cartesian_movement.base_vel[3:6])
            else:
                #针对后续处理
                vel_0 = lArm_link_vel_cart[i-1]
                omega_0 = lArm_link_omega_cart[i-1]

            # 前一级角速度变换到本帧
            omega_prev_in_i = temp_R @ omega_0

            #线速度: ⁱvᵢ = ⁱRᵢ₋₁·ⁱ⁻¹vᵢ₋₁ - ⁱω_prev_in_i × ⁱpᵢ₋₁
            temp_vel = temp_R @ vel_0 - np.cross(omega_prev_in_i, temp_P)
            #角速度: ⁱωᵢ = ⁱRᵢ₋₁·ⁱ⁻¹ωᵢ₋₁ + q̇ᵢ·ⁱzᵢ
            temp_omega = omega_prev_in_i + cho_vec_3D * arm_motor_movement.Joint_vel[i]

            lArm_link_vel_cart.append(temp_vel)
            lArm_link_omega_cart.append(temp_omega)

            # ----- 针对加速度处理 -----
            if i == 0:
                #针对第一级处理
                a_0 = np.array(base_Cartesian_movement.base_acc[0:3])
                dOmega_0 = np.array(base_Cartesian_movement.base_acc[3:6])
            else:
                #针对后续处理
                a_0 = lArm_link_acc_cart[i-1]
                dOmega_0 = lArm_link_dOmega_cart[i-1]

            dOmega_prev_in_i = temp_R @ dOmega_0

            #线加速度: ⁱaᵢ = ⁱRᵢ₋₁·ⁱ⁻¹aᵢ₋₁ - ⁱω̇_prev_in_i × ⁱpᵢ₋₁ - ⁱω_prev_in_i × (ⁱω_prev_in_i × ⁱpᵢ₋₁)
            temp_acc = temp_R @ a_0 - np.cross(dOmega_prev_in_i, temp_P) - np.cross(omega_prev_in_i, np.cross(omega_prev_in_i, temp_P))
            #角加速度: ⁱω̇ᵢ = ⁱRᵢ₋₁·ⁱ⁻¹ω̇ᵢ₋₁ + ⁱRᵢ₋₁·(ⁱ⁻¹ωᵢ₋₁ × q̇ᵢ·zᵢ) + q̈ᵢ·ⁱzᵢ
            temp_dOmega = dOmega_prev_in_i + temp_R @ np.cross(omega_0, cho_vec_3D * arm_motor_movement.Joint_vel[i]) + cho_vec_3D * arm_motor_movement.Joint_acc[i]

            #提取质心位置并计算质心线加速度: a_c = aᵢ + ω̇ᵢ × r_c + ωᵢ × (ωᵢ × r_c)
            frame_id = self.origin_robot.model.getFrameId(robot_arm_joint.lArm_frame_names[i])
            joint_id = self.origin_robot.model.frames[frame_id].parentJoint
            com = self.link_com_local[joint_id - 1]
            temp_acc_center = temp_acc + np.cross(temp_dOmega, com) + np.cross(temp_omega, np.cross(temp_omega, com))

            lArm_link_acc_cart.append(temp_acc)
            lArm_link_dOmega_cart.append(temp_dOmega)
            lArm_link_acc_cart_center.append(temp_acc_center)

        #针对右臂前向第推
        for i in range(robot_arm_joint.rArm_jointNum):
            if robot_arm_joint.rArm_JntRotation_axis[i] == 'x':
                cho_vec_3D = np.array([1, 0, 0])
            elif robot_arm_joint.rArm_JntRotation_axis[i] == 'y':
                cho_vec_3D = np.array([0, 1, 0])
            else:
                cho_vec_3D = np.array([0, 0, 1])

            temp_R = rArm_transmit_backward[i][:3, :3]
            temp_P = rArm_transmit_backward[i][:3, 3]

            # ----- 针对速度处理 -----
            if i == 0:
                #针对第一级处理 -- 已知世界坐标系0相对于自身的运动
                vel_0 = np.array(base_Cartesian_movement.base_vel[0:3])
                omega_0 = np.array(base_Cartesian_movement.base_vel[3:6])
            else:
                #针对后续处理
                vel_0 = rArm_link_vel_cart[i-1]
                omega_0 = rArm_link_omega_cart[i-1]

            # 前一级角速度变换到本帧
            omega_prev_in_i = temp_R @ omega_0

            #线速度: ⁱvᵢ = ⁱRᵢ₋₁·ⁱ⁻¹vᵢ₋₁ - ⁱω_prev_in_i × ⁱpᵢ₋₁
            temp_vel = temp_R @ vel_0 - np.cross(omega_prev_in_i, temp_P)
            #角速度: ⁱωᵢ = ⁱRᵢ₋₁·ⁱ⁻¹ωᵢ₋₁ + q̇ᵢ·ⁱzᵢ
            temp_omega = omega_prev_in_i + cho_vec_3D * arm_motor_movement.Joint_vel[robot_arm_joint.lArm_jointNum + i]

            rArm_link_vel_cart.append(temp_vel)
            rArm_link_omega_cart.append(temp_omega)

            # ----- 针对加速度处理 -----
            if i == 0:
                #针对第一级处理
                a_0 = np.array(base_Cartesian_movement.base_acc[0:3])
                dOmega_0 = np.array(base_Cartesian_movement.base_acc[3:6])
            else:
                #针对后续处理
                a_0 = rArm_link_acc_cart[i-1]
                dOmega_0 = rArm_link_dOmega_cart[i-1]

            dOmega_prev_in_i = temp_R @ dOmega_0

            #线加速度: ⁱaᵢ = ⁱRᵢ₋₁·ⁱ⁻¹aᵢ₋₁ - ⁱω̇_prev_in_i × ⁱpᵢ₋₁ - ⁱω_prev_in_i × (ⁱω_prev_in_i × ⁱpᵢ₋₁)
            temp_acc = temp_R @ a_0 - np.cross(dOmega_prev_in_i, temp_P) - np.cross(omega_prev_in_i, np.cross(omega_prev_in_i, temp_P))
            #角加速度: ⁱω̇ᵢ = ⁱRᵢ₋₁·ⁱ⁻¹ω̇ᵢ₋₁ + ⁱRᵢ₋₁·(ⁱ⁻¹ωᵢ₋₁ × q̇ᵢ·zᵢ) + q̈ᵢ·ⁱzᵢ
            temp_dOmega = dOmega_prev_in_i + temp_R @ np.cross(omega_0, cho_vec_3D * arm_motor_movement.Joint_vel[robot_arm_joint.lArm_jointNum + i]) + cho_vec_3D * arm_motor_movement.Joint_acc[robot_arm_joint.lArm_jointNum + i]

            #提取质心位置并计算质心线加速度: a_c = aᵢ + ω̇ᵢ × r_c + ωᵢ × (ωᵢ × r_c)
            frame_id = self.origin_robot.model.getFrameId(robot_arm_joint.rArm_frame_names[i])
            joint_id = self.origin_robot.model.frames[frame_id].parentJoint
            com = self.link_com_local[joint_id - 1]
            temp_acc_center = temp_acc + np.cross(temp_dOmega, com) + np.cross(temp_omega, np.cross(temp_omega, com))

            rArm_link_acc_cart.append(temp_acc)
            rArm_link_dOmega_cart.append(temp_dOmega)
            rArm_link_acc_cart_center.append(temp_acc_center)

        #返回求解的各连杆速度、加速度结果
        return lArm_link_vel_cart, lArm_link_omega_cart, lArm_link_acc_cart, lArm_link_dOmega_cart, lArm_link_acc_cart_center, \
        rArm_link_vel_cart, rArm_link_omega_cart, rArm_link_acc_cart, rArm_link_dOmega_cart, rArm_link_acc_cart_center

    #力求解接口 -- 直接用力雅可比将末端力映射到关节空间，不考虑关节动力学模型
    def RobotDyModel_TorEnd_to_Jnt(self,
            TorEnd_l, TorEnd_r,
            current_lr_arm_motor_q = None
    ):
        TorEnd_np_l = np.array(TorEnd_l).flatten()
        TorEnd_np_r = np.array(TorEnd_r).flatten()

        # 将机器人现阶段角度注入模型
        if current_lr_arm_motor_q is not None:
            q_current = np.array(current_lr_arm_motor_q).flatten()
        else:
            q_current = np.zeros(14)

        # 使用origin_robot模型进行操作
        model = self.origin_robot.model
        data = model.createData()

        # 注入关节角度并更新运动学
        pin.forwardKinematics(model, data, q_current)
        pin.updateFramePlacements(model, data)

        # 计算关节雅可比矩阵
        pin.computeJointJacobians(model, data, q_current)

        # 获取左右臂末端帧ID
        L_hand_id = model.getFrameId(self.lArm_EndLabel)
        R_hand_id = model.getFrameId(self.rArm_EndLabel)

        # 获取左右臂末端帧的雅可比矩阵（6×nq，世界对齐坐标系）
        J_l = pin.getFrameJacobian(model, data, L_hand_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        J_r = pin.getFrameJacobian(model, data, R_hand_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)

        # 提取左右臂末端受力（各6维：[fx, fy, fz, τx, τy, τz]）
        F_l = TorEnd_np_l
        F_r = TorEnd_np_r

        # 力雅可比映射：τ = J^T * F，截取单臂7维
        tau_l = (J_l.T @ F_l).flatten()[:7]
        tau_r = (J_r.T @ F_r).flatten()[7:]

        return tau_l, tau_r

    #力求解接口 -- 机器人动力学模型输出
    def RobotDyModel_outp(self,
        robot_arm_joint: RobotStructure,
        robotArmMovement: RobotAllLinkMovement,
        current_lr_arm_motor_q = None
    ):

        #通过pinocchio获取转换矩阵列表
        #反向第推转换矩阵 -- 从末端逐级向基座第推
        lArm_transmit_backward_prop = []
        rArm_transmit_backward_prop = []

        #将机器人现阶段角度复制到机器人模型中
        if current_lr_arm_motor_q is not None:
            q_current = np.array(current_lr_arm_motor_q).flatten()
        else:
            q_current = np.zeros(14)
        data = self.origin_robot.model.createData()
        pin.forwardKinematics(self.origin_robot.model, data, q_current)
        pin.updateFramePlacements(self.origin_robot.model, data)

        # 从origin_robot模型提取各级转换矩阵
        lArm_frame_ids = [self.origin_robot.model.getFrameId(name)
                          for name in robot_arm_joint.lArm_frame_names]
        rArm_frame_ids = [self.origin_robot.model.getFrameId(name)
                          for name in robot_arm_joint.rArm_frame_names]

        for i, fid in enumerate(lArm_frame_ids):
            if i == 0:
                # 第一级：world → link_0 的逆 = link_0 → world（反向第推方向）
                T_fwd = pinoSE3ToNp(data.oMf[fid])
                T_bwd = np.linalg.inv(T_fwd)
            else:
                # link_{i-1} → link_i
                T_fwd = pinoSE3ToNp(
                    data.oMf[lArm_frame_ids[i-1]].inverse() * data.oMf[fid]
                )
                T_bwd = np.linalg.inv(T_fwd)        # link_i → link_{i-1}（反向第推方向）
            lArm_transmit_backward_prop.append(T_bwd)

        for i, fid in enumerate(rArm_frame_ids):
            if i == 0:
                # 第一级：world → link_0 的逆 = link_0 → world（反向第推方向）
                T_fwd = pinoSE3ToNp(data.oMf[fid])
                T_bwd = np.linalg.inv(T_fwd)
            else:
                # link_{i-1} → link_i
                T_fwd = pinoSE3ToNp(
                    data.oMf[rArm_frame_ids[i-1]].inverse() * data.oMf[fid]
                )
                T_bwd = np.linalg.inv(T_fwd)        # link_i → link_{i-1}（反向第推方向）
            rArm_transmit_backward_prop.append(T_bwd)

        T_list_l = []
        W_list_l = []

        #进行反向第推过程
        for i in range(robot_arm_joint.lArm_jointNum):
            Joint_ID = i

            #获取六维度旋转轴
            if robot_arm_joint.lArm_JntRotation_axis[Joint_ID] == 'x':
                cho_vec_6D = np.array([0, 0, 0, 1, 0, 0]).reshape(6, 1)
            elif robot_arm_joint.lArm_JntRotation_axis[i] == 'y':
                cho_vec_6D = np.array([0, 0, 0, 0, 1, 0]).reshape(6, 1)
            else:
                cho_vec_6D = np.array([0, 0, 0, 0, 0, 1]).reshape(6, 1)

            T_column_List = []
            W_column_list = []
            
            # 重力补偿：将世界坐标系重力变换到本连杆坐标系，叠加到线加速度
            g_world = np.array([0.0, 0.0, 9.81])  # z轴向上
            R_link_to_world = data.oMf[lArm_frame_ids[Joint_ID]].rotation
            g_in_link = R_link_to_world.T @ g_world
            acc_with_g = robotArmMovement.lArm_acc[Joint_ID] + g_in_link

            #计算基础观测矩阵
            temp_L_omega = L_func(robotArmMovement.lArm_omega[Joint_ID])
            temp_L_dOmega = L_func(robotArmMovement.lArm_dOmega[Joint_ID])
            temp_L_acc = L_func(acc_with_g)
            temp_K_dOmega = K_func(robotArmMovement.lArm_dOmega[Joint_ID])

            temp_A_b1 = np.zeros((3, 6))
            temp_A_b2 = temp_L_dOmega + temp_L_omega@temp_L_omega
            temp_A_b3 = acc_with_g.reshape(3, 1)
            temp_A_b4 = temp_K_dOmega + temp_L_omega@temp_K_dOmega
            temp_A_b5 = -1*temp_L_acc
            temp_A_b6 = np.zeros((3, 1))

            temp_A_c1 = np.hstack((temp_A_b1, temp_A_b2, temp_A_b3))
            temp_A_c2 = np.hstack((temp_A_b4, temp_A_b5, temp_A_b6))
            temp_A = np.vstack((temp_A_c1, temp_A_c2))

            #计算观测矩阵
            temp_T = temp_A
            for j in range(robot_arm_joint.lArm_jointNum):
                if j < Joint_ID:
                    T_column_List.append(np.zeros((6, 10)))
                    W_column_list.append(np.zeros((1, 10)))

                elif j == Joint_ID:
                    temp_W = cho_vec_6D.T @ temp_T

                    T_column_List.append(temp_T)
                    W_column_list.append(temp_W)

                else:
                    temp_R = lArm_transmit_backward_prop[j - 1][:3, :3]
                    temp_P = lArm_transmit_backward_prop[j - 1][:3, 3]

                    temp_Q_b3 = L_func(temp_P)
                    temp_Q = np.vstack([np.hstack([temp_R, np.zeros((3, 3))]), np.hstack([temp_Q_b3, temp_R])])

                    temp_T = temp_Q @ temp_T
                    temp_W = cho_vec_6D.T @ temp_T
                    T_column_List.append(temp_T)
                    W_column_list.append(temp_W)

            T_list_l.append(T_column_List)
            W_list_l.append(W_column_list)

        temp_count_2 = 0
        for column in T_list_l:
            temp_count_1 = 0
            for T_arr in column:
                if temp_count_1 == 0:
                    T_column = T_arr
                    temp_count_1 = temp_count_1 + 1
                else:
                    T_column = np.hstack([T_column, T_arr])

            if temp_count_2 == 0:
                T_l = T_column
                temp_count_2 += 1
            else:
                T_l = np.vstack([T_l, T_column])

        temp_count_2 = 0
        for column in W_list_l:
            temp_count_1 = 0
            for W_arr in column:
                if temp_count_1 == 0:
                    W_column = W_arr
                    temp_count_1 = temp_count_1 + 1
                else:
                    W_column = np.hstack([W_column, W_arr])
            
            if temp_count_2 == 0:
                W_l = W_column
                temp_count_2 += 1
            else:
                W_l = np.vstack([W_l, W_column])

        #针对右臂观测矩阵递推计算
        T_list_r = []
        W_list_r = []

        #进行反向第推过程
        for i in range(robot_arm_joint.rArm_jointNum):
            Joint_ID = i

            #获取六维度旋转轴
            if robot_arm_joint.rArm_JntRotation_axis[Joint_ID] == 'x':
                cho_vec_6D = np.array([0, 0, 0, 1, 0, 0]).reshape(6, 1)
            elif robot_arm_joint.rArm_JntRotation_axis[Joint_ID] == 'y':
                cho_vec_6D = np.array([0, 0, 0, 0, 1, 0]).reshape(6, 1)
            else:
                cho_vec_6D = np.array([0, 0, 0, 0, 0, 1]).reshape(6, 1)

            T_column_List = []
            W_column_list = []

            # 重力补偿：将世界坐标系重力变换到本连杆坐标系，叠加到线加速度
            g_world = np.array([0.0, 0.0, 9.81])  # z轴向上
            R_link_to_world = data.oMf[rArm_frame_ids[Joint_ID]].rotation
            g_in_link = R_link_to_world.T @ g_world
            acc_with_g = robotArmMovement.rArm_acc[Joint_ID] + g_in_link

            #计算基础观测矩阵
            temp_L_omega = L_func(robotArmMovement.rArm_omega[Joint_ID])
            temp_L_dOmega = L_func(robotArmMovement.rArm_dOmega[Joint_ID])
            temp_L_acc = L_func(acc_with_g)
            temp_K_dOmega = K_func(robotArmMovement.rArm_dOmega[Joint_ID])

            temp_A_b1 = np.zeros((3, 6))
            temp_A_b2 = temp_L_dOmega + temp_L_omega@temp_L_omega
            temp_A_b3 = acc_with_g.reshape(3, 1)
            temp_A_b4 = temp_K_dOmega + temp_L_omega@temp_K_dOmega
            temp_A_b5 = -1*temp_L_acc
            temp_A_b6 = np.zeros((3, 1))

            temp_A_c1 = np.hstack((temp_A_b1, temp_A_b2, temp_A_b3))
            temp_A_c2 = np.hstack((temp_A_b4, temp_A_b5, temp_A_b6))
            temp_A = np.vstack((temp_A_c1, temp_A_c2))

            #计算观测矩阵
            temp_T = temp_A
            for j in range(robot_arm_joint.rArm_jointNum):
                if j < Joint_ID:
                    T_column_List.append(np.zeros((6, 10)))
                    W_column_list.append(np.zeros((1, 10)))

                elif j == Joint_ID:
                    temp_W = cho_vec_6D.T @ temp_T

                    T_column_List.append(temp_T)
                    W_column_list.append(temp_W)

                else:
                    temp_R = rArm_transmit_backward_prop[j - 1][:3, :3]
                    temp_P = rArm_transmit_backward_prop[j - 1][:3, 3]

                    temp_Q_b3 = L_func(temp_P)
                    temp_Q = np.vstack([np.hstack([temp_R, np.zeros((3, 3))]), np.hstack([temp_Q_b3, temp_R])])

                    temp_T = temp_Q @ temp_T
                    temp_W = cho_vec_6D.T @ temp_T
                    T_column_List.append(temp_T)
                    W_column_list.append(temp_W)

            T_list_r.append(T_column_List)
            W_list_r.append(W_column_list)

        temp_count_2 = 0
        for column in T_list_r:
            temp_count_1 = 0
            for T_arr in column:
                if temp_count_1 == 0:
                    T_column = T_arr
                    temp_count_1 = temp_count_1 + 1
                else:
                    T_column = np.hstack([T_column, T_arr])

            if temp_count_2 == 0:
                T_r = T_column
                temp_count_2 += 1
            else:
                T_r = np.vstack([T_r, T_column])

        temp_count_2 = 0
        for column in W_list_r:
            temp_count_1 = 0
            for W_arr in column:
                if temp_count_1 == 0:
                    W_column = W_arr
                    temp_count_1 = temp_count_1 + 1
                else:
                    W_column = np.hstack([W_column, W_arr])

            if temp_count_2 == 0:
                W_r = W_column
                temp_count_2 += 1
            else:
                W_r = np.vstack([W_r, W_column])

        #提取十维度惯性参数向量
        for i in range(robot_arm_joint.lArm_jointNum):
            #惯性参数向量
            temp_interia = self.link_inertia[i]
            temp_mass = self.link_mass[i]
            temp_com_local = self.link_com_local[i] 
            temp_P = np.array([
                temp_interia[0,0], temp_interia[0,1], temp_interia[0,2], temp_interia[1,1], temp_interia[1,2], temp_interia[2,2],
                temp_mass*temp_com_local[0], temp_mass*temp_com_local[1], temp_mass*temp_com_local[2], temp_mass
            ])

            if i == 0:
                P_vec_l = temp_P.reshape(-1, 1)
            else:
                P_vec_l = np.vstack([P_vec_l, temp_P.reshape(-1, 1)])

        for i in range(robot_arm_joint.rArm_jointNum):
            tj = i + robot_arm_joint.lArm_jointNum
            #惯性参数向量
            temp_interia = self.link_inertia[tj]
            temp_mass = self.link_mass[tj]
            temp_com_local = self.link_com_local[tj]
            temp_P = np.array([
                temp_interia[0,0], temp_interia[0,1], temp_interia[0,2], temp_interia[1,1], temp_interia[1,2], temp_interia[2,2],
                temp_mass*temp_com_local[0], temp_mass*temp_com_local[1], temp_mass*temp_com_local[2], temp_mass
            ])

            if i == 0:
                P_vec_r = temp_P.reshape(-1, 1)
            else:
                P_vec_r = np.vstack([P_vec_r, temp_P.reshape(-1, 1)])

        tau_sp_l = W_l @ P_vec_l
        tau_sp_r = W_r @ P_vec_r

        int_vec = [0, 0, 0, 0, 0, 0, 0]
        if self.node is not None:
            self.node.publish_once(
                tau_sp_l, tau_sp_r, int_vec
            )

        return tau_sp_l, tau_sp_r
    
    #力求解接口 -- 已知关节力，采用阻尼最小二乘法求解其在末端的映射（该函数中不考虑动力学模型的影响，只负责映射）
    def RobotDyModel_JntTor_to_EndForce(self,
            tau_l,
            tau_r,
            current_lr_arm_motor_q = None,
            damping_lambda = 0.01
    ):
        """
        利用阻尼最小二乘法将关节力矩映射到末端力和力矩，左右臂分开求解
        τ_l = J_lᵀ·F_l  →  F_l = (J_l·J_lᵀ + λ²I)⁻¹·J_l·τ_l
        τ_r = J_rᵀ·F_r  →  F_r = (J_r·J_rᵀ + λ²I)⁻¹·J_r·τ_r

        参数:
            tau_l: 左臂关节力矩贡献 (nq,)
            tau_r: 右臂关节力矩贡献 (nq,)
            current_lr_arm_motor_q: 当前关节角度 (nq,)
            damping_lambda: 阻尼因子λ
        返回:
            F_l: 左臂末端力和力矩 (6,)
            F_r: 右臂末端力和力矩 (6,)
        """
        tau_l_np = np.array(tau_l).flatten()
        tau_r_np = np.array(tau_r).flatten()

        # 将机器人现阶段角度注入模型
        if current_lr_arm_motor_q is not None:
            q_current = np.array(current_lr_arm_motor_q).flatten()
        else:
            q_current = np.array(self.origin_robot.q0).flatten()

        # 使用origin_robot模型进行操作
        model = self.origin_robot.model
        data = model.createData()

        # 注入关节角度并更新运动学
        pin.forwardKinematics(model, data, q_current)
        pin.updateFramePlacements(model, data)

        # 计算关节雅可比矩阵
        pin.computeJointJacobians(model, data, q_current)

        # 获取左右臂末端帧ID
        L_hand_id = model.getFrameId(self.lArm_EndLabel)
        R_hand_id = model.getFrameId(self.rArm_EndLabel)

        # 获取左右臂末端帧的雅可比矩阵（6×nq，世界对齐坐标系）
        J_l = pin.getFrameJacobian(model, data, L_hand_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        J_r = pin.getFrameJacobian(model, data, R_hand_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)

        # 截取单臂对应的雅可比列（左臂: 列0-6，右臂: 列7-13）
        J_l_7 = J_l[:, :7]   # 6×7
        J_r_7 = J_r[:, 7:]   # 6×7

        # 左右臂分别求解: F = (J·Jᵀ + λ²I)⁻¹·J·τ
        # 左臂
        A_l = J_l_7 @ J_l_7.T + (damping_lambda ** 2) * np.eye(6)
        b_l = J_l_7 @ tau_l_np
        F_l = np.linalg.solve(A_l, b_l)

        # 右臂
        A_r = J_r_7 @ J_r_7.T + (damping_lambda ** 2) * np.eye(6)
        b_r = J_r_7 @ tau_r_np
        F_r = np.linalg.solve(A_r, b_r)

        return F_l, F_r

    #力求解接口封装指南 -- 已知末端力，综合动力学模型求解关节力
    #力求解接口封装指南 -- 已知关节力，利用阻尼最小二乘法求解末端力，需要考虑机器人动力学模型对输出的影响

#===========================================================================================================================
#数据分析处理函数

    """
        正解目标位姿与结果输出函数
        其作用为对比求解得到的关节空间位姿在笛卡尔空间映射，以及目标笛卡尔空间位姿
    ------------------------------
    输入量：
        FramePosDes -- 目标位姿（笛卡尔空间）
        JntArrayRes -- 关节求解结果

    """
    def IKresExport(self, FramePosDes_l, FramePosDes_r, JntArrayRes, csv_file):

        JntArrayRes_mapped_l, JntArrayRes_mapped_r = self.solveFk(JntArrayRes)

        x_des_l, y_des_l, z_des_l, row_des_l, pitch_des_l, yaw_des_l = pose_matrix_to_euler(FramePosDes_l)
        x_des_r, y_des_r, z_des_r, row_des_r, pitch_des_r, yaw_des_r = pose_matrix_to_euler(FramePosDes_r)
        
        x_get_l = JntArrayRes_mapped_l.translation[0]
        y_get_l = JntArrayRes_mapped_l.translation[1]
        z_get_l = JntArrayRes_mapped_l.translation[2]
        euler_angles_l = pin.rpy.matrixToRpy(JntArrayRes_mapped_l.rotation)
        row_get_l, pitch_get_l, yaw_get_l = euler_angles_l[0], euler_angles_l[1], euler_angles_l[2]

        x_get_r = JntArrayRes_mapped_r.translation[0]
        y_get_r = JntArrayRes_mapped_r.translation[1]
        z_get_r = JntArrayRes_mapped_r.translation[2]
        euler_angles_r = pin.rpy.matrixToRpy(JntArrayRes_mapped_r.rotation)
        row_get_r, pitch_get_r, yaw_get_r = euler_angles_r[0], euler_angles_r[1], euler_angles_r[2]

        x_err_l = x_des_l - x_get_l
        y_err_l = y_des_l - y_get_l
        z_err_l = z_des_l - z_get_l
        row_err_l = row_des_l - row_get_l
        pitch_err_l = pitch_des_l - pitch_get_l
        yaw_err_l = yaw_des_l - yaw_get_l

        x_err_r = x_des_r - x_get_r
        y_err_r = y_des_r - y_get_r
        z_err_r = z_des_r - z_get_r
        row_err_r = row_des_r - row_get_r
        pitch_err_r = pitch_des_r - pitch_get_r
        yaw_err_r = yaw_des_r - yaw_get_r

        # 准备数据行
        data_row = [
            x_des_l, y_des_l, z_des_l, row_des_l, pitch_des_l, yaw_des_l,  # 期望值
            x_get_l, y_get_l, z_get_l, row_get_l, pitch_get_l, yaw_get_l,  # 实际值
            x_err_l, y_err_l, z_err_l, row_err_l, pitch_err_l, yaw_err_l,  # 误差
            x_des_r, y_des_r, z_des_r, row_des_r, pitch_des_r, yaw_des_r,  # 期望值
            x_get_r, y_get_r, z_get_r, row_get_r, pitch_get_r, yaw_get_r,  # 实际值
            x_err_r, y_err_r, z_err_r, row_err_r, pitch_err_r, yaw_err_r   # 误差
        ]
        
        # 检查文件是否存在
        file_exists = os.path.isfile(csv_file)
    
        # 打开文件并写入数据
        with open(csv_file, mode='a', newline='') as file:
            writer = csv.writer(file)
            
            # 如果文件不存在，先写入表头
            if not file_exists:
                headers = [
                    'x_des_l', 'y_des_l', 'z_des_l', 'row_des_l', 'pitch_des_l', 'yaw_des_l',
                    'x_get_l', 'y_get_l', 'z_get_l', 'row_get_l', 'pitch_get_l', 'yaw_get_l',
                    'x_err_l', 'y_err_l', 'z_err_l', 'row_err_l', 'pitch_err_l', 'yaw_err_l',
                    'x_des_r', 'y_des_r', 'z_des_r', 'row_des_r', 'pitch_des_r', 'yaw_des_r',
                    'x_get_r', 'y_get_r', 'z_get_r', 'row_get_r', 'pitch_get_r', 'yaw_get_r',
                    'x_err_r', 'y_err_r', 'z_err_r', 'row_err_r', 'pitch_err_r', 'yaw_err_r'
                ]
                writer.writerow(headers)
            
            # 写入数据行
            writer.writerow(data_row)

    
