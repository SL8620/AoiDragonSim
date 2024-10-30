
#pragma once

//Ϊɶ�������࣬���ǽṹ�壿��
//����˼·��Сstruct����class���������Ҫʹ��ros2��Ҫ������ֲ���ĳ�topic����DDS�ܺ���

#include "Pdo_Struct.h"

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

	//ʹ��union���ͣ���ʡ�ռ䣬�����ܴ�һ���������ܶ��ֽ���Ч�ռ�
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
};

//RoboDataBus�๹�캯��
RoboDataBus::RoboDataBus()
{

}

//RoboDataBus����������
RoboDataBus::~RoboDataBus()
{
}