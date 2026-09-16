#include "controller/ik_solver_wrapper.h"
#include <iostream>
#include <sstream>
#include <filesystem>
#include <cstdlib>
#include <dlfcn.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

std::string get_packages_path(const std::string packages, const std::string folder) {
    try {
        std::string pkg_share = ament_index_cpp::get_package_share_directory(packages);
        std::filesystem::path p = pkg_share;
        p /= folder;
        if (std::filesystem::exists(p)) return p.string();
    } catch (...) {
        // ignore and fallback
    }
    return "";
}

// 辅助函数：在 Python import 之前强制加载 workspace 自带的 libcasadi
// 并设置 CASADIPATH 环境变量，确保 CasADi 插件（如 IPOPT 求解器）也从 workspace 加载
//
// 背景：CasADi 插件（libcasadi_nlpsol_ipopt.so 等）不是普通的动态库依赖，
// 而是通过 PluginInterface::load_plugin() 用 dlopen 逐个加载。
// 其搜索顺序为：casadipath → CASADIPATH → LD_LIBRARY_PATH。
// Python 模块的位置（site-packages/casadi/）会被自动设为 casadipath，
// 本任务通过 TOWEL_OPENROBOTS_PREFIX 显式提供该目录。设置 CASADIPATH 可确保优先搜索正确的目录。
static void preloadWorkspaceLibcasadi() {
#ifdef __linux__
    try {
        std::string openrobots_share = ament_index_cpp::get_package_share_directory("openrobots_libs_hw");
        std::filesystem::path pkg_prefix = std::filesystem::path(openrobots_share).parent_path().parent_path();
        std::string lib_dir = (pkg_prefix / "lib").string();
        std::string lib_path = (pkg_prefix / "lib" / "libcasadi.so.3.7").string();

        // (1) dlopen libcasadi 主库，利用"同一 SONAME 只加载一次"确保后续 _casadi.so 不出错
        void* handle = dlopen(lib_path.c_str(), RTLD_NOW | RTLD_GLOBAL);
        if (handle) {
            std::cout << "[ik_solver] Preloaded workspace libcasadi: " << lib_path << std::endl;
        } else {
            std::cerr << "[ik_solver] WARNING: Failed to preload " << lib_path
                      << ": " << dlerror() << std::endl;
        }

        // (2) 设置 CASADIPATH，使得 CasADi plugin loader 先搜索 workspace lib
        //     防止回退到 /opt/openrobots/lib 中 ABI 不兼容的插件
        const char* existing_casadi = std::getenv("CASADIPATH");
        if (!existing_casadi || existing_casadi[0] == '\0') {
            setenv("CASADIPATH", lib_dir.c_str(), 1);
        } else {
            // 仅在尚未包含时才前置
            std::string casadi_str(existing_casadi);
            if (casadi_str.find(lib_dir) == std::string::npos) {
                std::string new_casadi = lib_dir + ":" + casadi_str;
                setenv("CASADIPATH", new_casadi.c_str(), 1);
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "[ik_solver] WARNING: Cannot locate workspace libcasadi for preload: "
                  << e.what() << std::endl;
    }
#endif
}

IKSolverWrapper::IKSolverWrapper()
    : pModule(nullptr), pIKSolverInstance(nullptr), initialized(false) {
    // 初始化Python解释器
    Py_Initialize();

    // 在 Python 做任何 import 之前，先 dlopen workspace 自带的 libcasadi
    // 这确保后续 _casadi.so 加载时使用兼容的版本
    preloadWorkspaceLibcasadi();

    PyRun_SimpleString("import sys");

    // 通过 ament_index 定位 openrobots_libs_hw 的 Python site-packages
    try {
        std::string openrobots_share = ament_index_cpp::get_package_share_directory("openrobots_libs_hw");
        // share/openrobots_libs → 前缀 (install/openrobots_libs_hw)
        std::filesystem::path pkg_prefix = std::filesystem::path(openrobots_share).parent_path().parent_path();
        std::string pyver = "python" + std::to_string(PY_MAJOR_VERSION) + "." + std::to_string(PY_MINOR_VERSION);
        std::string site_packages = (pkg_prefix / "lib" / pyver / "site-packages").string();
        std::string cmd = "sys.path.append(\"" + site_packages + "\")";
        PyRun_SimpleString(cmd.c_str());
    } catch (const std::exception& e) {
        // 回退：尝试环境变量
        const char* env = std::getenv("OPENROBOTS_PYTHON_PATH");
        if (env) {
            std::string cmd = std::string("sys.path.append(\"") + env + "\")";
            PyRun_SimpleString(cmd.c_str());
        }
    }
    PyGILState_STATE gstate = PyGILState_Ensure();

    // 初始化numpy
    initNumpy();
}

IKSolverWrapper::~IKSolverWrapper() {
    // 清理Python对象
    Py_XDECREF(pIKSolverInstance);
    Py_XDECREF(pModule);

    // 结束Python解释器
    Py_Finalize();
}

int IKSolverWrapper::initNumpy() {
    //手动实现初始化numpy数组
    if (_import_array() < 0) {
        PyErr_Print();
        PyErr_SetString(PyExc_ImportError, "numpy.core.multiarray failed to import");
        return -1;
    }
    return 0;
}

bool IKSolverWrapper::initialize(

    //模型限制参数
    const std::string& urdf_path,
    const std::vector<double>& initial_q,
    const std::vector<double>& desc_zeroPos,
    const std::vector<std::string>& arm_end_labels,
    bool use_joint_limits,
    const std::vector<std::string>& limited_joints,
    const std::vector<double>& limited_joint_angles,

    //模型信息打印参数
    bool display_joint_info,
    bool use_visualization,
    bool display_messages,

    //控制器控制参数
    const std::vector<double>& optiWeight,
    int template_times,
    double errTol,
    int ifPrintLevel,
    bool ifPrintOptiTime

) {
    try {
        // 导入Python模块

        // 必须在任何 Python API 调用之前初始化解释器
        if (!Py_IsInitialized()) {
            Py_Initialize();
            if (!Py_IsInitialized()) {
                fprintf(stderr, "Failed to initialize Python interpreter!\n");
                return false;
            }
        }

        // 1. 获取 Python 的 sys 模块
        PyObject* pSysModule = PyImport_ImportModule("sys");
        if (!pSysModule) {
            PyErr_Print();
            throw std::runtime_error("Failed to import sys module");
        }

        // 2. 获取 sys.path 对象
        PyObject* pSysPath = PyObject_GetAttrString(pSysModule, "path");
        Py_DECREF(pSysModule); // 释放 sys 模块
        if (!pSysPath) {
            PyErr_Print();
            throw std::runtime_error("Failed to get sys.path");
        }

        // 3. 双重保险：import ik_solver 之前再次确保 workspace libcasadi 已预加载
        //    利用 dlopen + RTLD_GLOBAL 机制，同一 SONAME 的库只加载一次，
        //    不依赖 LD_LIBRARY_PATH 或 RUNPATH 的搜索顺序
        preloadWorkspaceLibcasadi();

        // 4. 将 ik_solver.py 所在目录添加到 sys.path
        std::string ikSolverPath = get_packages_path("dual_arm_control_hw", "scripts");
        if (ikSolverPath.empty()) {
            throw std::runtime_error(
                "Cannot locate dual_arm_control_hw/scripts directory. "
                "Make sure 'dual_arm_control_hw' package is installed and "
                "'source install/setup.bash' has been run.");
        }
        PyObject* pPath = PyUnicode_FromString(ikSolverPath.c_str());
        if (!pPath) {
            Py_DECREF(pSysPath);
            PyErr_Print();
            throw std::runtime_error("Failed to create path string");
        }

        if (PyList_Append(pSysPath, pPath) == -1) {
            Py_DECREF(pSysPath);
            Py_DECREF(pPath);
            PyErr_Print();
            throw std::runtime_error("Failed to append path to sys.path");
        }

        PyObject* pName = PyUnicode_FromString("ik_solver");
        pModule = PyImport_Import(pName);

        if (!pModule) {
            PyErr_Print();
            throw std::runtime_error("Failed to load ik_solver module");
        }

        //1. 获取RobotModelDataWrap类
        PyObject* pRobotModelDataWrap = PyObject_GetAttrString(pModule, "RobotModelDataWrap");
        if (!pRobotModelDataWrap || !PyCallable_Check(pRobotModelDataWrap)) {
            throw std::runtime_error("Cannot find RobotModelDataWrap class");
        }

        // 创建RobotModelDataWrap实例
        PyObject* pModelArgs = PyTuple_New(2);

        // 第一个参数：model_path
        PyTuple_SetItem(pModelArgs, 0, PyUnicode_FromString(urdf_path.c_str()));

        // 第二个参数：ArmEndLabel
        PyObject* pArmEndLabel = stringVectorToPythonList(arm_end_labels);
        PyTuple_SetItem(pModelArgs, 1, pArmEndLabel);

        PyObject* pModelWrap = PyObject_CallObject(pRobotModelDataWrap, pModelArgs);
        Py_DECREF(pModelArgs);
        Py_DECREF(pRobotModelDataWrap);

        if (!pModelWrap) {
            PyErr_Print();
            throw std::runtime_error("Failed to create RobotModelDataWrap instance");
        }

        //2. 获取RobotModelLim类
        PyObject* pRobotModelLim = PyObject_GetAttrString(pModule, "RobotModelLim");
        if (!pRobotModelLim || !PyCallable_Check(pRobotModelLim)) {
            throw std::runtime_error("Cannot find RobotModelLim class");
        }

        // 创建RobotModelLim实例
        PyObject* pLimArgs = PyTuple_New(4);
        PyTuple_SetItem(pLimArgs, 0, PyBool_FromLong(use_joint_limits));

        // 受限关节列表
        PyObject* pLimJointList = stringVectorToPythonList(limited_joints);
        PyTuple_SetItem(pLimArgs, 1, pLimJointList);

        PyTuple_SetItem(pLimArgs, 2, PyBool_FromLong(display_joint_info));

        // 受限关节角度列表
        PyObject* pLimJointAngles = vectorToPythonList(limited_joint_angles);
        PyTuple_SetItem(pLimArgs, 3, pLimJointAngles);

        PyObject* pLimMsgWrap = PyObject_CallObject(pRobotModelLim, pLimArgs);
        Py_DECREF(pLimArgs);
        Py_DECREF(pRobotModelLim);

        if (!pLimMsgWrap) {
            PyErr_Print();
            throw std::runtime_error("Failed to create RobotModelLim instance");
        }

        //3. 获取OptiInte类
        PyObject* pOptiInte = PyObject_GetAttrString(pModule, "OptiInte");
        if (!pOptiInte || !PyCallable_Check(pOptiInte)) {
            throw std::runtime_error("Cannot find OptiInte class");
        }

        // 创建OptiInte实例
        PyObject* pOptiArgs = PyTuple_New(10);

        // 获取控制器参数 -- 显示传递5个参数，其他的使用默认值
        // 优化过程中的基坐标系与末端坐标系——采用默认值
        PyTuple_SetItem(pOptiArgs, 0, PyLong_FromLong(3));
        PyTuple_SetItem(pOptiArgs, 1, PyLong_FromLong(21));
        PyTuple_SetItem(pOptiArgs, 2, PyLong_FromLong(39));

        // 优化器权重设置——需要传入
        PyObject* pOptiWeight = PyList_New(optiWeight.size());  //创建python list对象
        for(size_t i = 0; i < optiWeight.size(); i++){
            PyList_SetItem(pOptiWeight, i, PyFloat_FromDouble(optiWeight[i]));
        }
        PyTuple_SetItem(pOptiArgs, 3, pOptiWeight);

        // 滤波器设置——使用默认值
        PyObject* pFtrWeight = PyList_New(0);
        PyTuple_SetItem(pOptiArgs, 4, pFtrWeight);
        PyTuple_SetItem(pOptiArgs, 5, PyLong_FromLong(14));

        // 优化器控制量设置
        PyTuple_SetItem(pOptiArgs, 6, PyLong_FromLong(template_times));
        PyTuple_SetItem(pOptiArgs, 7, PyFloat_FromDouble(errTol));
        PyTuple_SetItem(pOptiArgs, 8, PyLong_FromLong(ifPrintLevel));
        PyTuple_SetItem(pOptiArgs, 9, PyBool_FromLong(ifPrintOptiTime));

        PyObject* pOptiInteInstance = PyObject_CallObject(pOptiInte, pOptiArgs);

        Py_DECREF(pOptiArgs);
        Py_DECREF(pOptiInte);

        if (!pOptiInteInstance) {
            PyErr_Print();
            throw std::runtime_error("Failed to create OptiInte instance");
        }

        //4. 获取ViewMsg类
        PyObject* pViewMsg = PyObject_GetAttrString(pModule, "ViewMsg");
        if (!pViewMsg || !PyCallable_Check(pViewMsg)) {
            throw std::runtime_error("Cannot find ViewMsg class");
        }

        // 创建ViewMsg实例
        PyObject* pViewArgs = PyTuple_New(2);
        PyTuple_SetItem(pViewArgs, 0, PyBool_FromLong(use_visualization));
        PyTuple_SetItem(pViewArgs, 1, PyBool_FromLong(display_messages));
        PyObject* pViewMsgInstance = PyObject_CallObject(pViewMsg, pViewArgs);
        Py_DECREF(pViewArgs);
        Py_DECREF(pViewMsg);

        if (!pViewMsgInstance) {
            PyErr_Print();
            throw std::runtime_error("Failed to create ViewMsg instance");
        }

        //5. 初始关节角度
        PyObject* pInitialQ = vectorToPythonList(initial_q);

        //6. 笛卡尔空间初始位姿
        PyObject* pDescZeroPos = vectorToPythonList(desc_zeroPos);

        // 获取DualArmIK类
        PyObject* pDualArmIKClass = PyObject_GetAttrString(pModule, "DualArmIK");
        if (!pDualArmIKClass || !PyCallable_Check(pDualArmIKClass)) {
            throw std::runtime_error("Cannot find DualArmIK class");
        }

        // 创建DualArmIK实例
        PyObject* pInitArgs = PyTuple_New(6);
        PyTuple_SetItem(pInitArgs, 0, pModelWrap);
        PyTuple_SetItem(pInitArgs, 1, pInitialQ);
        PyTuple_SetItem(pInitArgs, 2, pDescZeroPos);
        PyTuple_SetItem(pInitArgs, 3, pLimMsgWrap);
        PyTuple_SetItem(pInitArgs, 4, pOptiInteInstance);
        PyTuple_SetItem(pInitArgs, 5, pViewMsgInstance);

        pIKSolverInstance = PyObject_CallObject(pDualArmIKClass, pInitArgs);
        Py_DECREF(pInitArgs);
        Py_DECREF(pDualArmIKClass);

        if (!pIKSolverInstance) {
            PyErr_Print();
            throw std::runtime_error("Failed to create DualArmIK instance");
        }

        //----------所有销毁统一放到最后执行----------

        //释放模型安装辅助参数
        Py_DECREF(pPath);   //释放路径字符串
        Py_DECREF(pName);   //释放名称字符串

        //1. 销毁pModelWrap -- 模型包装
        Py_DECREF(pModelWrap);

        //2. 销毁pLimMsgWrap -- 模型限制包装
        Py_DECREF(pLimMsgWrap);

        //3. 销毁机器人控制参数包装变量
        Py_DECREF(pOptiInteInstance);

        //4. 销毁机器人信息展示参数包装变量
        Py_DECREF(pViewMsgInstance);

        initialized = true;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "\n============================================" << std::endl;
        std::cerr << "FATAL: IKSolverWrapper::initialize() FAILED" << std::endl;
        std::cerr << "Reason: " << e.what() << std::endl;
        std::cerr << "============================================" << std::endl;
        return false;
    }
}

/*
求解逆解的封装代码

*/
std::pair<KDL::JntArray, KDL::JntArray> IKSolverWrapper::solveIK(
    KDL::Frame left_wrist_pose,
    KDL::Frame  right_wrist_pose,
    const std::vector<double>& current_q
) {
    if (!initialized) {
        throw std::runtime_error("IKSolver not initialized");
    }

    std::pair<KDL::JntArray, KDL::JntArray> dualArmJnt;

    try {
        // 获取solve_ik方法
        PyObject* pFunc = PyObject_GetAttrString(pIKSolverInstance, "solveIk_ListRet");
        if (!pFunc || !PyCallable_Check(pFunc)) {
            throw std::runtime_error("Cannot find solveIk_ListRet method");
        }

        // 准备参数
        PyObject* pArgs = PyTuple_New(current_q.empty() ? 2 : 3);

        // 左腕位姿 (4x4矩阵)
        PyObject* pLeftPose = poseToPythonObject(left_wrist_pose);
        PyTuple_SetItem(pArgs, 0, pLeftPose);

        // 右腕位姿 (4x4矩阵)
        PyObject* pRightPose = poseToPythonObject(right_wrist_pose);
        PyTuple_SetItem(pArgs, 1, pRightPose);

        // 当前关节角度 (可选)
        if (!current_q.empty()) {
            PyObject* pCurrentQ = vectorToPythonList(current_q);
            PyTuple_SetItem(pArgs, 2, pCurrentQ);
        }

        // 调用方法
        PyObject* pResult = PyObject_CallObject(pFunc, pArgs);
        Py_DECREF(pFunc);
        Py_DECREF(pArgs);

        if (!pResult) {
            PyErr_Print();
            throw std::runtime_error("solve_ik call failed");
        }

        // 转换结果
        std::vector<double> result = pythonListToVector(pResult);
        dualArmJnt = pyListToDualArmJntArray(pResult);

        Py_DECREF(pResult);

        return dualArmJnt;
    } catch (const std::exception& e) {
        std::cerr << "IK solving error: " << e.what() << std::endl;
        return dualArmJnt;
    }
}

std::pair<KDL::Frame, KDL::Frame> IKSolverWrapper::solveFK(
    KDL::JntArray lArm_pos, KDL::JntArray rArm_pos
) {
    if (!initialized) {
        throw std::runtime_error("IKSolver not initialized");
    }

    //输出结果包装：
    KDL::Frame lArmFrame;
    KDL::Frame rArmFrame;

    try {
        // 获取solveFk方法
        PyObject* pFunc = PyObject_GetAttrString(pIKSolverInstance, "solveFk_ListRet");
        if (!pFunc || !PyCallable_Check(pFunc)) {
            throw std::runtime_error("Cannot find solveFk_ListRet method");
        }

        // 准备参数
        PyObject* pQ = JntArrayToPyObject(lArm_pos, rArm_pos);
        PyObject* pArgs = PyTuple_New(1);
        PyTuple_SetItem(pArgs, 0, pQ);

        // 调用方法
        PyObject* pResult = PyObject_CallObject(pFunc, pArgs);
        Py_DECREF(pFunc);
        Py_DECREF(pArgs);

        if (!pResult) {
            PyErr_Print();
            throw std::runtime_error("solveFk call failed");
        }

        // 解析结果 (应该是包含两个4x4矩阵的元组)
        if (!PyTuple_Check(pResult) || PyTuple_Size(pResult) != 2) {
            throw std::runtime_error("Invalid solveFk result format");
        }

        // 解析左臂位姿
        PyObject* pLeftPose = PyTuple_GetItem(pResult, 0);
        lArmFrame = pyListToFrame(pLeftPose);
        // 解析右臂位姿
        PyObject* pRightPose = PyTuple_GetItem(pResult, 1);
        rArmFrame = pyListToFrame(pRightPose);

        Py_DECREF(pResult);
        return {lArmFrame, rArmFrame};

    } catch (const std::exception& e) {
        std::cerr << "FK solving error: " << e.what() << std::endl;
        return {lArmFrame, rArmFrame};
    }
}

bool IKSolverWrapper::updateJointLimits(
    bool use_limits,
    const std::vector<std::string>& limited_joints,
    const std::vector<double>& limited_joint_angles,
    bool display_messages
) {
    if (!initialized) {
        throw std::runtime_error("IKSolver not initialized");
    }

    try {
        // 获取RobotModelLim类
        PyObject* pRobotModelLim = PyObject_GetAttrString(pModule, "RobotModelLim");
        if (!pRobotModelLim || !PyCallable_Check(pRobotModelLim)) {
            throw std::runtime_error("Cannot find RobotModelLim class");
        }

        // 创建RobotModelLim实例
        PyObject* pLimArgs = PyTuple_New(4);
        PyTuple_SetItem(pLimArgs, 0, PyBool_FromLong(use_limits));

        // 受限关节列表
        PyObject* pLimJointList = stringVectorToPythonList(limited_joints);
        PyTuple_SetItem(pLimArgs, 1, pLimJointList);

        PyTuple_SetItem(pLimArgs, 2, PyBool_FromLong(display_messages));

        // 受限关节角度列表
        PyObject* pLimJointAngles = vectorToPythonList(limited_joint_angles);
        PyTuple_SetItem(pLimArgs, 3, pLimJointAngles);

        PyObject* pLimMsgWrap = PyObject_CallObject(pRobotModelLim, pLimArgs);
        Py_DECREF(pLimArgs);
        Py_DECREF(pRobotModelLim);

        if (!pLimMsgWrap) {
            PyErr_Print();
            throw std::runtime_error("Failed to create RobotModelLim instance");
        }

        // 获取JntLimReDef方法
        PyObject* pFunc = PyObject_GetAttrString(pIKSolverInstance, "JntLimReDef");
        if (!pFunc || !PyCallable_Check(pFunc)) {
            Py_DECREF(pLimMsgWrap);
            throw std::runtime_error("Cannot find JntLimReDef method");
        }

        // 调用方法
        PyObject* pArgs = PyTuple_New(1);
        PyTuple_SetItem(pArgs, 0, pLimMsgWrap);
        PyObject* pResult = PyObject_CallObject(pFunc, pArgs);
        Py_DECREF(pFunc);
        Py_DECREF(pArgs);

        if (!pResult) {
            PyErr_Print();
            throw std::runtime_error("JntLimReDef call failed");
        }

        bool success = PyObject_IsTrue(pResult);
        Py_DECREF(pResult);
        return success;
    } catch (const std::exception& e) {
        std::cerr << "Update joint limits error: " << e.what() << std::endl;
        return false;
    }
}

RobotLinkMovement IKSolverWrapper::RobotDyModel_linkMovement(
            RobotJntMsg robotJntStruct,
            RobotJntMovement JntMove,
            RobotBaseMovement baseMove
){
    RobotLinkMovement result;

    if (!initialized) {
        throw std::runtime_error("IKSolver not initialized");
    }

    try {
        // 获取Python方法
        PyObject* pFunc = PyObject_GetAttrString(pIKSolverInstance, "RobotDyModel_linkMovement");
        if (!pFunc || !PyCallable_Check(pFunc)) {
            throw std::runtime_error("Cannot find RobotDyModel_linkMovement method");
        }

        // ========== 1. 构建 RobotStructure Python 对象 ==========
        PyObject* pRobotStructureClass = PyObject_GetAttrString(pModule, "RobotStructure");
        PyObject* pStructArgs = PyTuple_New(6);
        PyTuple_SetItem(pStructArgs, 0, stringVectorToPythonList(robotJntStruct.lArm_frame_names));
        PyTuple_SetItem(pStructArgs, 1, stringVectorToPythonList(robotJntStruct.rArm_frame_names));
        PyTuple_SetItem(pStructArgs, 2, stringVectorToPythonList(robotJntStruct.lArm_JntRotation_axis));
        PyTuple_SetItem(pStructArgs, 3, stringVectorToPythonList(robotJntStruct.rArm_JntRotation_axis));
        PyTuple_SetItem(pStructArgs, 4, PyLong_FromLong(robotJntStruct.lArm_JntNum));
        PyTuple_SetItem(pStructArgs, 5, PyLong_FromLong(robotJntStruct.rArm_JntNum));
        PyObject* pRobotStructure = PyObject_CallObject(pRobotStructureClass, pStructArgs);
        Py_DECREF(pStructArgs);
        Py_DECREF(pRobotStructureClass);

        if (!pRobotStructure) {
            PyErr_Print();
            throw std::runtime_error("Failed to create RobotStructure instance");
        }

        // ========== 2. 构建 RobotJointMovement Python 对象 ==========
        std::vector<double> allJointPos = JntMove.lArm_Joint_pos;
        allJointPos.insert(allJointPos.end(), JntMove.rArm_Joint_pos.begin(), JntMove.rArm_Joint_pos.end());
        std::vector<double> allJointVel = JntMove.lArm_Joint_vel;
        allJointVel.insert(allJointVel.end(), JntMove.rArm_Joint_vel.begin(), JntMove.rArm_Joint_vel.end());
        std::vector<double> allJointAcc = JntMove.lArm_Joint_acc;
        allJointAcc.insert(allJointAcc.end(), JntMove.rArm_Joint_acc.begin(), JntMove.rArm_Joint_acc.end());

        PyObject* pJointMovementClass = PyObject_GetAttrString(pModule, "RobotJointMovement");
        PyObject* pJointArgs = PyTuple_New(3);
        PyTuple_SetItem(pJointArgs, 0, vectorToPythonList(allJointPos));
        PyTuple_SetItem(pJointArgs, 1, vectorToPythonList(allJointVel));
        PyTuple_SetItem(pJointArgs, 2, vectorToPythonList(allJointAcc));
        PyObject* pJointMovement = PyObject_CallObject(pJointMovementClass, pJointArgs);
        Py_DECREF(pJointMovementClass);
        Py_DECREF(pJointArgs);

        if (!pJointMovement) {
            Py_DECREF(pRobotStructure);
            PyErr_Print();
            throw std::runtime_error("Failed to create RobotJointMovement instance");
        }

        // ========== 3. 构建 RobotBaseMovement Python 对象 ==========
        PyObject* pBaseMovementClass = PyObject_GetAttrString(pModule, "RobotBaseMovement");
        PyObject* pBaseArgs = PyTuple_New(3);
        PyTuple_SetItem(pBaseArgs, 0, vectorToPythonList(baseMove.base_pos));
        PyTuple_SetItem(pBaseArgs, 1, vectorToPythonList(baseMove.base_vel));
        PyTuple_SetItem(pBaseArgs, 2, vectorToPythonList(baseMove.base_acc));
        PyObject* pBaseMovement = PyObject_CallObject(pBaseMovementClass, pBaseArgs);
        Py_DECREF(pBaseMovementClass);
        Py_DECREF(pBaseArgs);

        if (!pBaseMovement) {
            Py_DECREF(pRobotStructure);
            Py_DECREF(pJointMovement);
            PyErr_Print();
            throw std::runtime_error("Failed to create RobotBaseMovement instance");
        }

        // ========== 4. 调用 Python 方法 ==========
        PyObject* pArgs = PyTuple_New(3);
        PyTuple_SetItem(pArgs, 0, pRobotStructure);
        PyTuple_SetItem(pArgs, 1, pJointMovement);
        PyTuple_SetItem(pArgs, 2, pBaseMovement);

        PyObject* pResult = PyObject_CallObject(pFunc, pArgs);
        Py_DECREF(pFunc);
        Py_DECREF(pArgs);

        if (!pResult) {
            PyErr_Print();
            throw std::runtime_error("RobotDyModel_linkMovement call failed");
        }

        // ========== 5. 解析返回值：10元组 -> RobotLinkMovement ==========
        if (!PyTuple_Check(pResult) || PyTuple_Size(pResult) != 10) {
            Py_DECREF(pResult);
            throw std::runtime_error("Invalid RobotDyModel_linkMovement result: expected tuple of 10");
        }

        auto pyListOfArraysToVecVec = [](PyObject* pyList) -> std::vector<std::vector<double>> {
            std::vector<std::vector<double>> out;
            if (!PyList_Check(pyList)) return out;

            Py_ssize_t size = PyList_Size(pyList);
            out.reserve(size);
            for (Py_ssize_t i = 0; i < size; i++) {
                PyObject* item = PyList_GetItem(pyList, i);
                std::vector<double> vec;
                if (PyArray_Check(item)) {
                    double* data = (double*)PyArray_DATA((PyArrayObject*)item);
                    npy_intp len = PyArray_SIZE((PyArrayObject*)item);
                    vec.assign(data, data + len);
                } else if (PyList_Check(item)) {
                    Py_ssize_t innerSize = PyList_Size(item);
                    vec.reserve(innerSize);
                    for (Py_ssize_t j = 0; j < innerSize; j++) {
                        vec.push_back(PyFloat_AsDouble(PyList_GetItem(item, j)));
                    }
                }
                out.push_back(std::move(vec));
            }
            return out;
        };

        result.lArm_vel        = pyListOfArraysToVecVec(PyTuple_GetItem(pResult, 0));
        result.lArm_omega      = pyListOfArraysToVecVec(PyTuple_GetItem(pResult, 1));
        result.lArm_acc        = pyListOfArraysToVecVec(PyTuple_GetItem(pResult, 2));
        result.lArm_domega     = pyListOfArraysToVecVec(PyTuple_GetItem(pResult, 3));
        result.lArm_acc_center = pyListOfArraysToVecVec(PyTuple_GetItem(pResult, 4));
        result.rArm_vel        = pyListOfArraysToVecVec(PyTuple_GetItem(pResult, 5));
        result.rArm_omega      = pyListOfArraysToVecVec(PyTuple_GetItem(pResult, 6));
        result.rArm_acc        = pyListOfArraysToVecVec(PyTuple_GetItem(pResult, 7));
        result.rArm_domega     = pyListOfArraysToVecVec(PyTuple_GetItem(pResult, 8));
        result.rArm_acc_center = pyListOfArraysToVecVec(PyTuple_GetItem(pResult, 9));

        Py_DECREF(pResult);

        return result;

    } catch (const std::exception& e) {
        std::cerr << "RobotDyModel_linkMovement error: " << e.what() << std::endl;
        return result;
    }
}

RobotJntTor IKSolverWrapper::Dy_TorEnd_toJnt(
    std::vector<double> TorEnd_l, std::vector<double> TorEnd_r,
    KDL::JntArray lArm_JntPos, KDL::JntArray rArm_JntPos
){
    RobotJntTor result;

    if (!initialized) {
        throw std::runtime_error("IKSolver not initialized");
    }

    try {
        PyObject* pFunc = PyObject_GetAttrString(pIKSolverInstance, "RobotDyModel_TorEnd_to_Jnt");
        if (!pFunc || !PyCallable_Check(pFunc)) {
            throw std::runtime_error("Cannot find RobotDyModel_TorEnd_to_Jnt method");
        }

        std::vector<double> currentQ;
        currentQ.reserve(lArm_JntPos.rows() + rArm_JntPos.rows());
        for (unsigned int i = 0; i < lArm_JntPos.rows(); ++i)
            currentQ.push_back(lArm_JntPos(i));
        for (unsigned int i = 0; i < rArm_JntPos.rows(); ++i)
            currentQ.push_back(rArm_JntPos(i));

        PyObject* pArgs = PyTuple_New(3);
        PyTuple_SetItem(pArgs, 0, vectorToPythonList(TorEnd_l));
        PyTuple_SetItem(pArgs, 1, vectorToPythonList(TorEnd_r));
        PyTuple_SetItem(pArgs, 2, vectorToPythonList(currentQ));

        PyObject* pResult = PyObject_CallObject(pFunc, pArgs);
        Py_DECREF(pFunc);
        Py_DECREF(pArgs);

        if (!pResult) {
            PyErr_Print();
            throw std::runtime_error("RobotDyModel_TorEnd_to_Jnt call failed");
        }

        if (!PyTuple_Check(pResult) || PyTuple_Size(pResult) != 2) {
            Py_DECREF(pResult);
            throw std::runtime_error("Invalid RobotDyModel_TorEnd_to_Jnt result: expected tuple of 2");
        }

        PyObject* pTauL = PyTuple_GetItem(pResult, 0);
        if (PyArray_Check(pTauL)) {
            double* data = (double*)PyArray_DATA((PyArrayObject*)pTauL);
            npy_intp size = PyArray_SIZE((PyArrayObject*)pTauL);
            result.lArm_JntTor.assign(data, data + size);
        } else if (PyList_Check(pTauL)) {
            result.lArm_JntTor = pythonListToVector(pTauL);
        }

        PyObject* pTauR = PyTuple_GetItem(pResult, 1);
        if (PyArray_Check(pTauR)) {
            double* data = (double*)PyArray_DATA((PyArrayObject*)pTauR);
            npy_intp size = PyArray_SIZE((PyArrayObject*)pTauR);
            result.rArm_JntTor.assign(data, data + size);
        } else if (PyList_Check(pTauR)) {
            result.rArm_JntTor = pythonListToVector(pTauR);
        }

        Py_DECREF(pResult);
        return result;

    } catch (const std::exception& e) {
        std::cerr << "Dy_TorEnd_toJnt error: " << e.what() << std::endl;
        return result;
    }
}

RobotJntTor IKSolverWrapper::RobotDyModelOutp(
    RobotJntMsg robotJntStruct,
    RobotLinkMovement linkMovement,
    KDL::JntArray lArm_JntPos, KDL::JntArray rArm_JntPos
){
    RobotJntTor result;

    if (!initialized) {
        throw std::runtime_error("IKSolver not initialized");
    }

    try {
        PyObject* pFunc = PyObject_GetAttrString(pIKSolverInstance, "RobotDyModel_outp");
        if (!pFunc || !PyCallable_Check(pFunc)) {
            throw std::runtime_error("Cannot find RobotDyModel_outp method");
        }

        auto vecVecToPyListOfArrays = [](const std::vector<std::vector<double>>& src) -> PyObject* {
            PyObject* pyList = PyList_New(src.size());
            for (size_t i = 0; i < src.size(); i++) {
                npy_intp dim = static_cast<npy_intp>(src[i].size());
                PyObject* arr = PyArray_SimpleNew(1, &dim, NPY_DOUBLE);
                double* data = (double*)PyArray_DATA((PyArrayObject*)arr);
                std::copy(src[i].begin(), src[i].end(), data);
                PyList_SetItem(pyList, i, arr);
            }
            return pyList;
        };

        PyObject* pRobotStructureClass = PyObject_GetAttrString(pModule, "RobotStructure");
        PyObject* pStructArgs = PyTuple_New(6);
        PyTuple_SetItem(pStructArgs, 0, stringVectorToPythonList(robotJntStruct.lArm_frame_names));
        PyTuple_SetItem(pStructArgs, 1, stringVectorToPythonList(robotJntStruct.rArm_frame_names));
        PyTuple_SetItem(pStructArgs, 2, stringVectorToPythonList(robotJntStruct.lArm_JntRotation_axis));
        PyTuple_SetItem(pStructArgs, 3, stringVectorToPythonList(robotJntStruct.rArm_JntRotation_axis));
        PyTuple_SetItem(pStructArgs, 4, PyLong_FromLong(robotJntStruct.lArm_JntNum));
        PyTuple_SetItem(pStructArgs, 5, PyLong_FromLong(robotJntStruct.rArm_JntNum));
        PyObject* pRobotStructure = PyObject_CallObject(pRobotStructureClass, pStructArgs);
        Py_DECREF(pStructArgs);
        Py_DECREF(pRobotStructureClass);

        if (!pRobotStructure) {
            PyErr_Print();
            throw std::runtime_error("Failed to create RobotStructure instance");
        }

        PyObject* pLinkMovementClass = PyObject_GetAttrString(pModule, "RobotAllLinkMovement");
        PyObject* pLinkArgs = PyTuple_New(10);
        PyTuple_SetItem(pLinkArgs, 0, vecVecToPyListOfArrays(linkMovement.lArm_vel));
        PyTuple_SetItem(pLinkArgs, 1, vecVecToPyListOfArrays(linkMovement.lArm_omega));
        PyTuple_SetItem(pLinkArgs, 2, vecVecToPyListOfArrays(linkMovement.lArm_acc));
        PyTuple_SetItem(pLinkArgs, 3, vecVecToPyListOfArrays(linkMovement.lArm_domega));
        PyTuple_SetItem(pLinkArgs, 4, vecVecToPyListOfArrays(linkMovement.lArm_acc_center));
        PyTuple_SetItem(pLinkArgs, 5, vecVecToPyListOfArrays(linkMovement.rArm_vel));
        PyTuple_SetItem(pLinkArgs, 6, vecVecToPyListOfArrays(linkMovement.rArm_omega));
        PyTuple_SetItem(pLinkArgs, 7, vecVecToPyListOfArrays(linkMovement.rArm_acc));
        PyTuple_SetItem(pLinkArgs, 8, vecVecToPyListOfArrays(linkMovement.rArm_domega));
        PyTuple_SetItem(pLinkArgs, 9, vecVecToPyListOfArrays(linkMovement.rArm_acc_center));

        PyObject* pLinkMovement = PyObject_CallObject(pLinkMovementClass, pLinkArgs);
        Py_DECREF(pLinkMovementClass);
        Py_DECREF(pLinkArgs);

        if (!pLinkMovement) {
            Py_DECREF(pRobotStructure);
            PyErr_Print();
            throw std::runtime_error("Failed to create RobotAllLinkMovement instance");
        }

        std::vector<double> currentQ;
        currentQ.reserve(lArm_JntPos.rows() + rArm_JntPos.rows());
        for (unsigned int i = 0; i < lArm_JntPos.rows(); ++i)
            currentQ.push_back(lArm_JntPos(i));
        for (unsigned int i = 0; i < rArm_JntPos.rows(); ++i)
            currentQ.push_back(rArm_JntPos(i));

        PyObject* pArgs = PyTuple_New(3);
        PyTuple_SetItem(pArgs, 0, pRobotStructure);
        PyTuple_SetItem(pArgs, 1, pLinkMovement);
        PyTuple_SetItem(pArgs, 2, vectorToPythonList(currentQ));

        PyObject* pResult = PyObject_CallObject(pFunc, pArgs);
        Py_DECREF(pFunc);
        Py_DECREF(pArgs);

        if (!pResult) {
            PyErr_Print();
            throw std::runtime_error("RobotDyModel_outp call failed");
        }

        if (!PyTuple_Check(pResult) || PyTuple_Size(pResult) != 2) {
            Py_DECREF(pResult);
            throw std::runtime_error("Invalid RobotDyModel_outp result: expected tuple of 2");
        }

        PyObject* pTauL = PyTuple_GetItem(pResult, 0);
        if (PyArray_Check(pTauL)) {
            double* data = (double*)PyArray_DATA((PyArrayObject*)pTauL);
            npy_intp size = PyArray_SIZE((PyArrayObject*)pTauL);
            result.lArm_JntTor.assign(data, data + size);
        } else if (PyList_Check(pTauL)) {
            result.lArm_JntTor = pythonListToVector(pTauL);
        }

        PyObject* pTauR = PyTuple_GetItem(pResult, 1);
        if (PyArray_Check(pTauR)) {
            double* data = (double*)PyArray_DATA((PyArrayObject*)pTauR);
            npy_intp size = PyArray_SIZE((PyArrayObject*)pTauR);
            result.rArm_JntTor.assign(data, data + size);
        } else if (PyList_Check(pTauR)) {
            result.rArm_JntTor = pythonListToVector(pTauR);
        }

        Py_DECREF(pResult);
        return result;

    } catch (const std::exception& e) {
        std::cerr << "RobotDyModelOutp error: " << e.what() << std::endl;
        return result;
    }
}

RobotFrameTor IKSolverWrapper::Dy_JntTor_toEnd(
    RobotJntTor JntTor,
    KDL::JntArray lArm_JntPos, KDL::JntArray rArm_JntPos,
    double damping_lambda
){
    RobotFrameTor result;

    if (!initialized) {
        throw std::runtime_error("IKSolver not initialized");
    }

    try {
        PyObject* pFunc = PyObject_GetAttrString(pIKSolverInstance, "RobotDyModel_JntTor_to_EndForce");
        if (!pFunc || !PyCallable_Check(pFunc)) {
            throw std::runtime_error("Cannot find RobotDyModel_JntTor_to_EndForce method");
        }

        std::vector<double> currentQ;
        currentQ.reserve(lArm_JntPos.rows() + rArm_JntPos.rows());
        for (unsigned int i = 0; i < lArm_JntPos.rows(); ++i)
            currentQ.push_back(lArm_JntPos(i));
        for (unsigned int i = 0; i < rArm_JntPos.rows(); ++i)
            currentQ.push_back(rArm_JntPos(i));

        PyObject* pArgs = PyTuple_New(4);
        PyTuple_SetItem(pArgs, 0, vectorToPythonList(JntTor.lArm_JntTor));
        PyTuple_SetItem(pArgs, 1, vectorToPythonList(JntTor.rArm_JntTor));
        PyTuple_SetItem(pArgs, 2, vectorToPythonList(currentQ));
        PyTuple_SetItem(pArgs, 3, PyFloat_FromDouble(damping_lambda));

        PyObject* pResult = PyObject_CallObject(pFunc, pArgs);
        Py_DECREF(pFunc);
        Py_DECREF(pArgs);

        if (!pResult) {
            PyErr_Print();
            throw std::runtime_error("RobotDyModel_JntTor_to_EndForce call failed");
        }

        if (!PyTuple_Check(pResult) || PyTuple_Size(pResult) != 2) {
            Py_DECREF(pResult);
            throw std::runtime_error("Invalid RobotDyModel_JntTor_to_EndForce result: expected tuple of 2");
        }

        PyObject* pFL = PyTuple_GetItem(pResult, 0);
        if (PyArray_Check(pFL)) {
            double* data = (double*)PyArray_DATA((PyArrayObject*)pFL);
            npy_intp size = PyArray_SIZE((PyArrayObject*)pFL);
            result.lArm_EndTor.assign(data, data + size);
        } else if (PyList_Check(pFL)) {
            result.lArm_EndTor = pythonListToVector(pFL);
        }

        PyObject* pFR = PyTuple_GetItem(pResult, 1);
        if (PyArray_Check(pFR)) {
            double* data = (double*)PyArray_DATA((PyArrayObject*)pFR);
            npy_intp size = PyArray_SIZE((PyArrayObject*)pFR);
            result.rArm_EndTor.assign(data, data + size);
        } else if (PyList_Check(pFR)) {
            result.rArm_EndTor = pythonListToVector(pFR);
        }

        Py_DECREF(pResult);
        return result;

    } catch (const std::exception& e) {
        std::cerr << "Dy_JntTor_toEnd error: " << e.what() << std::endl;
        return result;
    }
}

//--------------------------------------------------------------------------------------
// 辅助函数实现
PyObject* IKSolverWrapper::vectorToPythonList(const std::vector<double>& vec) {
    PyObject* list = PyList_New(vec.size());
    for (size_t i = 0; i < vec.size(); ++i) {
        PyList_SetItem(list, i, PyFloat_FromDouble(vec[i]));
    }
    return list;
}

PyObject* IKSolverWrapper::stringVectorToPythonList(const std::vector<std::string>& vec) {
    PyObject* list = PyList_New(vec.size());
    for (size_t i = 0; i < vec.size(); ++i) {
        PyList_SetItem(list, i, PyUnicode_FromString(vec[i].c_str()));
    }
    return list;
}

std::vector<double> IKSolverWrapper::pythonListToVector(PyObject* pyList) {
    std::vector<double> vec;
    if (PyList_Check(pyList)) {
        Py_ssize_t size = PyList_Size(pyList);
        vec.reserve(size);
        for (Py_ssize_t i = 0; i < size; ++i) {
            PyObject* item = PyList_GetItem(pyList, i);
            vec.push_back(PyFloat_AsDouble(item));
        }
    } else if (PyArray_Check(pyList)) {
        double* data = (double*)PyArray_DATA((PyArrayObject*)pyList);
        npy_intp size = PyArray_SIZE((PyArrayObject*)pyList);
        vec.assign(data, data + size);
    }
    return vec;
}

std::vector<std::string> IKSolverWrapper::pythonListToStringVector(PyObject* pyList) {
    std::vector<std::string> vec;
    if (PyList_Check(pyList)) {
        Py_ssize_t size = PyList_Size(pyList);
        vec.reserve(size);
        for (Py_ssize_t i = 0; i < size; ++i) {
            PyObject* item = PyList_GetItem(pyList, i);
            const char* str = PyUnicode_AsUTF8(item);
            if (str) {
                vec.emplace_back(str);
            }
        }
    }
    return vec;
}

std::pair<KDL::JntArray, KDL::JntArray> IKSolverWrapper::pyListToDualArmJntArray(PyObject* pyList){
    KDL::JntArray lArmJntArray;
    KDL::JntArray rArmJntArray;
    lArmJntArray.resize(LARMDOF);
    rArmJntArray.resize(RARMDOF);

    if(PyList_Check(pyList)){
        Py_ssize_t size = PyList_Size(pyList);
        if(size != LARMDOF + RARMDOF){
            std::cerr << "错误：机器人求解器返回结果自由度数目与模型关节数目不匹配" <<
                "机器人求解器返回自由度数目：" << size <<
                "模型关节数目：" << LARMDOF + RARMDOF << std::endl;
                return {lArmJntArray, rArmJntArray};
        }

        for (int i = 0; i < LARMDOF; ++i) {
            PyObject* item = PyList_GetItem(pyList, i);
            lArmJntArray(i) = PyFloat_AsDouble(item);
        }
        for (int i = 0; i < RARMDOF; ++i) {
            PyObject* item = PyList_GetItem(pyList, LARMDOF + i);
            rArmJntArray(i) = PyFloat_AsDouble(item);
        }

        return {lArmJntArray, rArmJntArray};
    }else{
        std::cerr << "错误：关节角度值应当返回类型为列表" << std::endl;
        return {lArmJntArray, rArmJntArray};
    }
}

KDL::Frame IKSolverWrapper::pyListToFrame(PyObject* pyList){
    if (!PyList_Check(pyList) && !PyTuple_Check(pyList)) {
        throw std::runtime_error("Input is not a list or tuple");
    }

    Py_ssize_t size = PySequence_Size(pyList);
    if (size != 16) {
        throw std::runtime_error("Input list must have 16 elements (4x4 matrix)");
    }

    std::vector<double> matrixData;
    for (Py_ssize_t i = 0; i < size; ++i) {
        PyObject* item = PySequence_GetItem(pyList, i);
        if (!PyFloat_Check(item) && !PyLong_Check(item)) {
            Py_DECREF(item);
            throw std::runtime_error("List elements must be numbers");
        }
        matrixData.push_back(PyFloat_AsDouble(item));
        Py_DECREF(item);
    }

    KDL::Rotation rotation;
    KDL::Vector translation;

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            rotation(i, j) = matrixData[i * 4 + j];
        }
        translation[i] = matrixData[i * 4 + 3];
    }

    return KDL::Frame(rotation, translation);
}

PyObject* IKSolverWrapper::poseToPythonObject(KDL::Frame pose) {
    npy_intp dims[2] = {4, 4};
    PyObject* obj = PyArray_SimpleNew(2, dims, NPY_DOUBLE);
    double* data = (double*)PyArray_DATA((PyArrayObject*)obj);

    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            data[i*4 + j] = pose.M(i, j);
        }
    }
    data[0*4 + 3] = pose.p.x();
    data[1*4 + 3] = pose.p.y();
    data[2*4 + 3] = pose.p.z();
    data[3*4 + 0] = 0;
    data[3*4 + 1] = 0;
    data[3*4 + 2] = 0;
    data[3*4 + 3] = 1;

    return obj;
}

void IKSolverWrapper::optiWeight_def(
    int optiWeight_mod,
    const std::vector<double>& optiWeight_selfDefine,
    bool ifUseFtr
){
    try{
        PyObject* pFunc = PyObject_GetAttrString(pIKSolverInstance, "optiWeight_update");
        if (!pFunc || !PyCallable_Check(pFunc)) {
            throw std::runtime_error("Cannot find optiWeight_update method");
        }

        PyObject* pArgs = PyTuple_New(3);
        PyTuple_SetItem(pArgs, 0, PyLong_FromLong(optiWeight_mod));
        PyTuple_SetItem(pArgs, 1, vectorToPythonList(optiWeight_selfDefine));
        PyTuple_SetItem(pArgs, 2, PyBool_FromLong(ifUseFtr));

        PyObject* pResult = PyObject_CallObject(pFunc, pArgs);
        if (!pResult) {
            PyErr_Print();
            throw std::runtime_error("optiWeight_update call failed");
        }

        Py_DECREF(pResult);
        Py_DECREF(pFunc);
        Py_DECREF(pArgs);

    }catch(const std::exception& e) {
        std::cerr << "optimize Weight update error: " << e.what() << std::endl;
    }
}

PyObject* IKSolverWrapper::JntArrayToPyObject(KDL::JntArray lArm_pos, KDL::JntArray rArm_pos){
    PyObject* list = PyList_New(lArm_pos.rows() + rArm_pos.rows());

    size_t temp = 0;
    for(size_t i = 0; i < lArm_pos.rows(); i++){
        PyList_SetItem(list, temp, PyFloat_FromDouble(lArm_pos(i)));
        temp += 1;
    }
    for(size_t j = 0; j < rArm_pos.rows(); j++){
        PyList_SetItem(list, temp, PyFloat_FromDouble(rArm_pos(j)));
        temp += 1;
    }
    return list;
}

bool IKSolverWrapper::checkPythonCall(PyObject* pResult, const std::string& funcName) {
    if (!pResult) {
        PyErr_Print();
        std::cerr << "Error calling Python function: " << funcName << std::endl;
        return false;
    }
    return true;
}
