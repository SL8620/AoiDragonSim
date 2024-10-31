/*** 
 * @Author: SL8620
 * @Date: 2024-10-30 11:17:28
 * @LastEditTime: 2024-10-30 23:13:37
 * @LastEditors: SL8620
 * @Description: 
 * @FilePath: \AoiDragonSim\RoboDataBus\RoboDatabus.h
 * @��������Ԥ���İ�Ȩ����������ǩ�������е�
 */



#pragma once

//Ϊɶ�������࣬���ǽṹ�壿��
//����˼·��Сstruct����class���������Ҫʹ��ros2��Ҫ������ֲ���ĳ�topic����DDS�ܺ���
//�²�24/10/30���Ժ�д���뾡����Ҫ����д����������ע��Ҳ��Ҫ��Ӣ��
//�²�24/10/30����������Ⱥ�˳�򿴵����ܣ�����һ��
//�²�24/10/30�������̬����ôһ��rotһ��RPY�ģ��ѵ��еĵ�����ʾ��ת��������������ʾŷ������

#include "Pdo_Struct.h"
#include <iostream>
#include <vector>
#include "Eigen/Dense"
#include "iomanip"


//--------------------------------���������Ϣ�ṹ��--------------------------------

enum class DriveStatus	//������״̬ö������
{
	PreOP = 0,	//PreOPģʽ�����ڴ�ģʽ�½���PDOӳ������
	Start = 1,	//Startģʽ���ɽ���PDO��SDOָ��
	Stop = 2	//Stopģʽ��ֹͣ����ָ��
};

enum class PdoMapType	//PDOӳ���������
{
	CST_Mode = 1,	//��׼ģʽ
	STD_Mode = 2	//CSTģʽ
};

struct DriveDataBus
{
	DriveStatus m_DriveStatus;				//������״̬
	PdoMapType	m_PdoMapType;				//������PDOӳ������

	//ʹ��union���ͣ���ʡ�ռ䣬�����ܴ�һ����������ܶ���Ч�ռ�
	union 
	{
		TPDO_STD_Data m_TPDO_STD_Data;  // TPDO����׼ģʽ������
		TPDO_CST_Data m_TPDO_CST_Data;  // TPDO��CSTģʽ������
	};
	union 
	{
		RPDO_STD_Data m_RPDO_STD_Data;  // RPDO����׼ģʽ������
		RPDO_CST_Data m_RPDO_CST_Data;  // RPDO��CSTģʽ������
	};
	bool isCSTMode;                     // ģʽ��ʶ����true ��ʾ CST ģʽ��false ��ʾ��׼ģʽ
};

class RoboDataBus
{
public:
	RoboDataBus();
	~RoboDataBus();

private:
	DriveDataBus m_DriveDataBus;	//�����ṹ������

	const int model_nv;		//nv��ʲô��˼��number of Velocity?������ע����number of dq

	const Eigen::Matrix3d feet_Left_Rotation_Local_offset=(Eigen::MatrixXd(3,3) << 1,0,0, 0,1,0, 0,0,1).finished();		//��ų�ʼ��̬������ڻ���������ϵ
	const Eigen::Matrix3d feet_Right_Rotation_Local_offset=(Eigen::MatrixXd(3,3) << 1,0,0, 0,1,0, 0,0,1).finished();	//�ҽų�ʼ��̬������ڻ���������ϵ

	//�������������״̬���� �ǵò�ȫ˳��
	double RPY[3];				//RPY�ǣ�Ŀǰ����֪���Ǹ�˭�õģ��ǵò�ȫ
	double fL[3];				//feetLeft?�ǵò�ȫ
	double fR[3];				//feetRight?�ǵò�ȫ
	double basePos[3];			//����baselink������
	double baseLinearVel[3];	//�������ٶ�
	double baseLinearAcc[3];	//�����߼��ٶ�
	double baseAngularVel[3];	//�������ٶ�
	std::vector<double>	motor_current_position;		//�����ǰλ��
	std::vector<double>	motor_current_velocity;		//�����ǰ�ٶ�
	std::vector<double>	motor_current_torque;		//�����ǰ����
	Eigen::VectorXd FL_estimation,FR_estimation;	//������д���⵽����ɶ��ForceLeft?
	bool isdqIni;									//�����Ǹ�ɶ�ı�־λ

	//PVT����(��Ϊ��֪��PVT�Ͳ�дȫ���ˣ���֪���Ĺ���Ҳ�ܲ³���)
	std::vector<double> motors_desired_position;	//�������λ��
    std::vector<double> motors_desired_velocity;	//��������ٶ�
    std::vector<double> motors_desired_torque;		//�������Ť��
    std::vector<double> motors_output_torque;		//���ʵ��Ť��������������motor_current_torque��ɶ����

	//״̬�͹ؼ�����
	Eigen::VectorXd q, dq, ddq;										//λ�ã��ٶȣ����ٶ�
    Eigen::VectorXd qOld;											//�ϴ�λ��
    Eigen::MatrixXd J_base, J_l, J_r, J_hd_l, J_hd_r, J_hip_link;	//�����������ȣ������֣��Źؽڵ��ſɱ�
    Eigen::MatrixXd dJ_base, dJ_l, dJ_r, dJ_hd_l, dJ_hd_r;			//�ſɱ�΢�֣�
    Eigen::MatrixXd Jcom_W; 										//�����ſɱȣ���������ϵ
    Eigen::Vector3d pCoM_W;											//����λ�ã���������ϵ
    Eigen::Vector3d fe_r_pos_W, fe_l_pos_W, base_pos;				//���ҽ�λ�ã���������ϵ+����λ�ã�
    Eigen::Matrix3d fe_r_rot_W, fe_l_rot_W, base_rot; 				//���ҽ���̬����������ϵ+������̬��
    Eigen::Vector3d fe_r_pos_L, fe_l_pos_L;							//���ҽ�λ�ã�����������ϵ
	Eigen::Matrix3d fe_r_rot_L, fe_l_rot_L;							//���ҽ���̬������������ϵ
    Eigen::Vector3d hip_link_pos;									//�Źؽ�λ�ã�ɶ����ϵ	
	Eigen::Matrix3d hip_link_rot;									//�Źؽ���̬	
    Eigen::Vector3d hip_r_pos_L, hip_l_pos_L;						//�����Źؽ�λ�ã�����������ϵ
    Eigen::Vector3d hip_r_pos_W, hip_l_pos_W;						//�����Źؽ�λ�ã���������ϵ
    Eigen::Vector3d fe_r_pos_L_cmd, fe_l_pos_L_cmd;					//���ҽ����λ���������������ϵ
    Eigen::Matrix3d fe_r_rot_L_cmd, fe_l_rot_L_cmd;					//���ҽ������̬�������������ϵ

    Eigen::Vector3d hd_r_pos_W, hd_l_pos_W;							//������λ�ã���������ϵ	
    Eigen::Matrix3d hd_r_rot_W, hd_l_rot_W;							//��������̬����������ϵ
    Eigen::Vector3d hd_r_pos_L, hd_l_pos_L;							//������λ�ã�����������ϵ
    Eigen::Matrix3d hd_r_rot_L, hd_l_rot_L;							//��������̬������������ϵ
    Eigen::VectorXd qCmd, dqCmd;									//λ������ٶ�����
    Eigen::VectorXd tauJointCmd;									//�ؽ���������
    Eigen::MatrixXd dyn_M, dyn_M_inv, dyn_C, dyn_Ag, dyn_dAg;		//����ѧ��ز����������������������C���Ƕȣ����ٶ�
    Eigen::VectorXd dyn_G, dyn_Non;									//����ѧ��ز�����������Non
    Eigen::Vector3d base_omega_L, base_omega_W, base_rpy;			//�������ٶȣ�����������ϵ�����������ٶȣ���������ϵ��,������̬

    Eigen::Vector3d slop;											//��϶����ɶ��
    Eigen::Matrix<double,3,3> inertia;								//���Ծ���

	//���ֱ���ȡ�Ŀ���ֵ
	Eigen::Vector3d     joystick_desired_euler;							//������̬����				
    Eigen::Vector3d     joystick_desired_position;						//����λ������
    Eigen::Vector3d     joystick_desired_omega;							//�������ٶ�����
    Eigen::Vector3d     joystick_desired_velocity;						//�����ٶ�����

	//MPC����ó��Ŀ���ֵ
    Eigen::VectorXd     Xd;												//X����ֵ��X��ɶ������
    Eigen::VectorXd     X_cur;											//X����ֵ
	// Eigen::Vector3d     mpc_eul_des;
	// Eigen::Vector3d     mpc_pos_des;
	// Eigen::Vector3d     mpc_omega_des;
	// Eigen::Vector3d     mpc_vel_des;
    Eigen::VectorXd     X_cal;											//X����ֵ
    Eigen::VectorXd     dX_cal;											//dX����ֵ
    Eigen::VectorXd     feedback_react_tau_cmd;							//����������ֵ��
	int 	qp_nWSR_MPC;
    double 	qp_cpuTime_MPC;
    int 	qpStatus_MPC;

	//WBC����ó��Ŀ���ֵ
	Eigen::Vector3d base_rpy_des;									//����������̬
    Eigen::Vector3d base_pos_des;									//��������λ��
    Eigen::VectorXd des_ddq, des_dq, des_delta_q, des_q;			//�������ٶȣ��ٶȣ�λ�ñ仯����λ��
    Eigen::Vector3d swing_fe_pos_des_W;								//�ڶ�������λ�ã���������ϵ
    Eigen::Vector3d swing_fe_rot_des_W;								//�ڶ���������̬����������ϵ
    Eigen::Vector3d stance_fe_pos_cur_W;							//վ���ŵ�ǰλ�ã���������ϵ
    Eigen::Matrix3d stance_fe_rot_cur_W;							//վ���ŵ�ǰ��̬����������ϵ
    Eigen::VectorXd wbc_delta_q_final, wbc_dq_final, wbc_ddq_final;	//WBC������Ħ�q,dq,ddq
    Eigen::VectorXd wbc_tauJointRes;								//WBC����������ؽ��
    Eigen::VectorXd wbc_FrRes;										//WBC������������������
    Eigen::VectorXd Fr_ff;											//����ɶ��
    int qp_nWSR_WBC;
    double qp_cpuTime_WBC;
    int qp_status_WBC;
	
	//��̬�滮
    Eigen::Vector3d swingStartPos_W;								//�ڶ�����ʼλ�ã���������ϵ
    Eigen::Vector3d swingDesPosCur_W;								//�ڶ�������λ�ã���������ϵ
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

//RoboDataBus�๹�캯��
RoboDataBus::RoboDataBus()
{

}

//RoboDataBus����������
RoboDataBus::~RoboDataBus()
{
}