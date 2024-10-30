/*** 
 * @Author: SL8620
 * @Date: 2024-10-30 11:17:28
 * @LastEditTime: 2024-10-30 22:15:19
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
    Eigen::MatrixXd Jcom_W; // jacobian of CoM, in world frame		//�����ſɱȣ���������ϵ
    Eigen::Vector3d pCoM_W;											//����λ�ã���������ϵ
    Eigen::Vector3d fe_r_pos_W, fe_l_pos_W, base_pos;				//���ҽ�λ�ã���������ϵ+����λ�ã�
    Eigen::Matrix3d fe_r_rot_W, fe_l_rot_W, base_rot; 				//���ҽ���̬����������ϵ+������̬��
    Eigen::Vector3d fe_r_pos_L, fe_l_pos_L; // in Body frame		//���ҽ�λ�ã�����������ϵ
    Eigen::Vector3d hip_link_pos;									
    Eigen::Vector3d hip_r_pos_L, hip_l_pos_L;						
    Eigen::Vector3d hip_r_pos_W, hip_l_pos_W;
    Eigen::Matrix3d fe_r_rot_L, fe_l_rot_L;
    Eigen::Matrix3d hip_link_rot;
    Eigen::Vector3d fe_r_pos_L_cmd, fe_l_pos_L_cmd;
    Eigen::Matrix3d fe_r_rot_L_cmd, fe_l_rot_L_cmd;

    Eigen::Vector3d hd_r_pos_W, hd_l_pos_W; // in world frame
    Eigen::Matrix3d hd_r_rot_W, hd_l_rot_W;
    Eigen::Vector3d hd_r_pos_L, hd_l_pos_L; // in body frame
    Eigen::Matrix3d hd_r_rot_L, hd_l_rot_L;
    Eigen::VectorXd qCmd, dqCmd;
    Eigen::VectorXd tauJointCmd;
    Eigen::MatrixXd dyn_M, dyn_M_inv, dyn_C, dyn_Ag, dyn_dAg;
    Eigen::VectorXd dyn_G, dyn_Non;
    Eigen::Vector3d base_omega_L, base_omega_W, base_rpy;

    Eigen::Vector3d slop;
    Eigen::Matrix<double,3,3>   inertia;

};

//RoboDataBus�๹�캯��
RoboDataBus::RoboDataBus()
{

}

//RoboDataBus����������
RoboDataBus::~RoboDataBus()
{
}