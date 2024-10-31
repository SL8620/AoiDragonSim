/*** 
 * @Author: SL8620
 * @Date: 2024-10-30 11:17:28
 * @LastEditTime: 2024-10-30 23:13:37
 * @LastEditors: SL8620
 * @Description: 
 * @FilePath: \AoiDragonSim\RoboDataBus\RoboDatabus.h
 * @可以输入预定的版权声明、个性签名、空行等
 */



#pragma once

//为啥他不用类，而是结构体？？
//个人思路：小struct进大class，后续如果要使用ros2需要大量移植，改成topic，用DDS很合适
//吐槽24/10/30：以后写代码尽量不要用缩写，看不懂，注释也不要用英文
//吐槽24/10/30：这个命名先后顺序看得难受，改了一下
//吐槽24/10/30：这个姿态角怎么一会rot一会RPY的，难道有的单独表示旋转矩阵，其他单单表示欧拉角吗

#include "Pdo_Struct.h"
#include <iostream>
#include <vector>
#include "Eigen/Dense"
#include "iomanip"


//--------------------------------电机驱动信息结构体--------------------------------

enum class DriveStatus	//驱动器状态枚举类型
{
	PreOP = 0,	//PreOP模式，可在此模式下进行PDO映射配置
	Start = 1,	//Start模式，可接受PDO和SDO指令
	Stop = 2	//Stop模式，停止接收指令
};

enum class PdoMapType	//PDO映射具体类型
{
	CST_Mode = 1,	//标准模式
	STD_Mode = 2	//CST模式
};

struct DriveDataBus
{
	DriveStatus m_DriveStatus;				//驱动器状态
	PdoMapType	m_PdoMapType;				//驱动器PDO映射类型

	//使用union类型，节省空间，否则会很大，一个电机会多出很多无效空间
	union 
	{
		TPDO_STD_Data m_TPDO_STD_Data;  // TPDO：标准模式下数据
		TPDO_CST_Data m_TPDO_CST_Data;  // TPDO：CST模式下数据
	};
	union 
	{
		RPDO_STD_Data m_RPDO_STD_Data;  // RPDO：标准模式下数据
		RPDO_CST_Data m_RPDO_CST_Data;  // RPDO：CST模式下数据
	};
	bool isCSTMode;                     // 模式标识符：true 表示 CST 模式，false 表示标准模式
};

class RoboDataBus
{
public:
	RoboDataBus();
	~RoboDataBus();

private:
	DriveDataBus m_DriveDataBus;	//驱动结构体类型

	const int model_nv;		//nv是什么意思？number of Velocity?他给的注释是number of dq

	const Eigen::Matrix3d feet_Left_Rotation_Local_offset=(Eigen::MatrixXd(3,3) << 1,0,0, 0,1,0, 0,0,1).finished();		//左脚初始姿态，相对于机器人坐标系
	const Eigen::Matrix3d feet_Right_Rotation_Local_offset=(Eigen::MatrixXd(3,3) << 1,0,0, 0,1,0, 0,0,1).finished();	//右脚初始姿态，相对于机器人坐标系

	//电机、传感器、状态反馈 记得补全顺序
	double RPY[3];				//RPY角，目前还不知道是给谁用的，记得补全
	double fL[3];				//feetLeft?记得补全
	double fR[3];				//feetRight?记得补全
	double basePos[3];			//基座baselink的坐标
	double baseLinearVel[3];	//基座线速度
	double baseLinearAcc[3];	//基座线加速度
	double baseAngularVel[3];	//基座角速度
	std::vector<double>	motor_current_position;		//电机当前位置
	std::vector<double>	motor_current_velocity;		//电机当前速度
	std::vector<double>	motor_current_torque;		//电机当前力矩
	Eigen::VectorXd FL_estimation,FR_estimation;	//又是缩写，这到底是啥，ForceLeft?
	bool isdqIni;									//这又是干啥的标志位

	//PVT控制(广为人知的PVT就不写全称了，不知道的估计也能猜出来)
	std::vector<double> motors_desired_position;	//电机期望位置
    std::vector<double> motors_desired_velocity;	//电机期望速度
    std::vector<double> motors_desired_torque;		//电机期望扭矩
    std::vector<double> motors_output_torque;		//电机实际扭矩输出，和上面的motor_current_torque有啥区别

	//状态和关键变量
	Eigen::VectorXd q, dq, ddq;										//位置，速度，加速度
    Eigen::VectorXd qOld;											//上次位置
    Eigen::MatrixXd J_base, J_l, J_r, J_hd_l, J_hd_r, J_hip_link;	//基座，左右腿？左右手？髋关节的雅可比
    Eigen::MatrixXd dJ_base, dJ_l, dJ_r, dJ_hd_l, dJ_hd_r;			//雅可比微分？
    Eigen::MatrixXd Jcom_W; 										//质心雅可比，世界坐标系
    Eigen::Vector3d pCoM_W;											//质心位置，世界坐标系
    Eigen::Vector3d fe_r_pos_W, fe_l_pos_W, base_pos;				//左右脚位置，世界坐标系+基座位置？
    Eigen::Matrix3d fe_r_rot_W, fe_l_rot_W, base_rot; 				//左右脚姿态，世界坐标系+基座姿态？
    Eigen::Vector3d fe_r_pos_L, fe_l_pos_L;							//左右脚位置，机器人坐标系
	Eigen::Matrix3d fe_r_rot_L, fe_l_rot_L;							//左右脚姿态，机器人坐标系
    Eigen::Vector3d hip_link_pos;									//髋关节位置，啥坐标系	
	Eigen::Matrix3d hip_link_rot;									//髋关节姿态	
    Eigen::Vector3d hip_r_pos_L, hip_l_pos_L;						//左右髋关节位置，机器人坐标系
    Eigen::Vector3d hip_r_pos_W, hip_l_pos_W;						//左右髋关节位置，世界坐标系
    Eigen::Vector3d fe_r_pos_L_cmd, fe_l_pos_L_cmd;					//左右脚落脚位置命令，机器人坐标系
    Eigen::Matrix3d fe_r_rot_L_cmd, fe_l_rot_L_cmd;					//左右脚落脚姿态命令，机器人坐标系

    Eigen::Vector3d hd_r_pos_W, hd_l_pos_W;							//左右手位置，世界坐标系	
    Eigen::Matrix3d hd_r_rot_W, hd_l_rot_W;							//左右手姿态，世界坐标系
    Eigen::Vector3d hd_r_pos_L, hd_l_pos_L;							//左右手位置，机器人坐标系
    Eigen::Matrix3d hd_r_rot_L, hd_l_rot_L;							//左右手姿态，机器人坐标系
    Eigen::VectorXd qCmd, dqCmd;									//位置命令，速度命令
    Eigen::VectorXd tauJointCmd;									//关节力矩命令
    Eigen::MatrixXd dyn_M, dyn_M_inv, dyn_C, dyn_Ag, dyn_dAg;		//动力学相关参数，质量矩阵，质量逆矩阵，C，角度，角速度
    Eigen::VectorXd dyn_G, dyn_Non;									//动力学相关参数，重力，Non
    Eigen::Vector3d base_omega_L, base_omega_W, base_rpy;			//基座角速度（机器人坐标系），基座角速度（世界坐标系）,基座姿态

    Eigen::Vector3d slop;											//间隙？干啥的
    Eigen::Matrix<double,3,3> inertia;								//惯性矩阵

	//从手柄获取的控制值
	Eigen::Vector3d     joystick_desired_euler;							//期望姿态命令				
    Eigen::Vector3d     joystick_desired_position;						//期望位置命令
    Eigen::Vector3d     joystick_desired_omega;							//期望角速度命令
    Eigen::Vector3d     joystick_desired_velocity;						//期望速度命令

	//MPC计算得出的控制值
    Eigen::VectorXd     Xd;												//X期望值（X是啥东西）
    Eigen::VectorXd     X_cur;											//X现在值
	// Eigen::Vector3d     mpc_eul_des;
	// Eigen::Vector3d     mpc_pos_des;
	// Eigen::Vector3d     mpc_omega_des;
	// Eigen::Vector3d     mpc_vel_des;
    Eigen::VectorXd     X_cal;											//X计算值
    Eigen::VectorXd     dX_cal;											//dX计算值
    Eigen::VectorXd     feedback_react_tau_cmd;							//力反馈控制值？
	int 	qp_nWSR_MPC;
    double 	qp_cpuTime_MPC;
    int 	qpStatus_MPC;

	//WBC计算得出的控制值
	Eigen::Vector3d base_rpy_des;									//基座期望姿态
    Eigen::Vector3d base_pos_des;									//基座期望位置
    Eigen::VectorXd des_ddq, des_dq, des_delta_q, des_q;			//期望加速度，速度，位置变化量，位置
    Eigen::Vector3d swing_fe_pos_des_W;								//摆动脚期望位置，世界坐标系
    Eigen::Vector3d swing_fe_rot_des_W;								//摆动脚期望姿态，世界坐标系
    Eigen::Vector3d stance_fe_pos_cur_W;							//站立脚当前位置，世界坐标系
    Eigen::Matrix3d stance_fe_rot_cur_W;							//站立脚当前姿态，世界坐标系
    Eigen::VectorXd wbc_delta_q_final, wbc_dq_final, wbc_ddq_final;	//WBC计算出的δq,dq,ddq
    Eigen::VectorXd wbc_tauJointRes;								//WBC计算出的力矩结果
    Eigen::VectorXd wbc_FrRes;										//WBC计算出的力反馈结果？
    Eigen::VectorXd Fr_ff;											//这是啥？
    int qp_nWSR_WBC;
    double qp_cpuTime_WBC;
    int qp_status_WBC;
	
	//步态规划
    Eigen::Vector3d swingStartPos_W;								//摆动脚起始位置，世界坐标系
    Eigen::Vector3d swingDesPosCur_W;								//摆动脚期望位置，世界坐标系
    Eigen::Vector3d swingDesPosCur_L;
    Eigen::Vector3d swingDesPosFinal_W;
    Eigen::Vector3d stanceDesPos_W;
    Eigen::Vector3d posHip_W, posST_W;
    Eigen::Vector3d desV_W; // desired linear velocity
    double desWz_W; // desired angular velocity
    double theta0; // offset yaw angle of the swing leg, w.r.t body frame
    double width_hips; // distance between the left and right hip
    double tSwing;
    double phi;

};

//RoboDataBus类构造函数
RoboDataBus::RoboDataBus()
{

}

//RoboDataBus类析构函数
RoboDataBus::~RoboDataBus()
{
}