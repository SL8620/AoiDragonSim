
#pragma once

//为啥他不用类，而是结构体？？
//个人思路：小struct进大class，后续如果要使用ros2需要大量移植，改成topic，用DDS很合适

#include "Pdo_Struct.h"

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

	//使用union类型，节省空间，否则会很大，一个电机多出很多字节无效空间
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
};

//RoboDataBus类构造函数
RoboDataBus::RoboDataBus()
{

}

//RoboDataBus类析构函数
RoboDataBus::~RoboDataBus()
{
}