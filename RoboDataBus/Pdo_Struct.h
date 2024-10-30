/*** 
 * @Author: SL8620
 * @Date: 2024-10-29 19:59:47
 * @LastEditTime: 2024-10-30 10:45:31
 * @LastEditors: SL8620
 * @Description: 
 * @FilePath: \AzureLoongSim\RoboDataBus\Pdo_Struct.h
 * @可以输入预定的版权声明、个性签名、空行等
 */

#pragma once

//TPDO传输方向：驱动端（Elmo）->控制端（STM32）
struct TPDO_STD_Mode
{
    int actualPosition;     // 实际位置 (32位)，设备当前的实际物理位置
    int actualVelocity;     // 实际速度 (32位)，设备当前的实际运行速度
    int actualTorque;       // 实际扭矩 (16位)，设备当前输出的实际扭矩
    int statusword;         // 状态字 (16位)，表示设备的当前状态，包括运行、错误等信息
    int actualCurrent;      // 实际电流 (16位)，电机当前的实际电流
};
struct TPDO_CST_Mode
{
    int targetPosition;     // 目标位置 (32位)，用于控制电机或设备的位置
    int targetVelocity;     // 目标速度 (32位)，用于控制电机或设备的速度
    int targetTorque;       // 目标扭矩 (16位)，用于控制设备的扭矩输出
    int maxTorque;          // 最大扭矩(16位)，表示允许的最大扭矩限制
    int controlWord;        // 控制字 (16位)，用于控制状态机的运行（如启停、复位）
    int modeOfOperation;    // 操作模式 (8位)，表示当前的控制模式 (如位置模式、速度模式等)
    int torqueOffset;       // 扭矩偏置 (16位)，用于补偿扭矩的偏移量
};
//RPDO传输方向：控制端（STM32）->驱动端（Elmo）
struct RPDO_STD_Mode
{
    int targetPosition;     // 目标位置 (32位)，用于控制电机或设备的位置
    int targetVelocity;     // 目标速度 (32位)，用于控制电机或设备的速度
    int targetTorque;       // 目标扭矩 (16位)，用于控制设备的扭矩输出
    int maxTorque;          // 最大扭矩(16位)，表示允许的最大扭矩限制
    int controlWord;        // 控制字 (16位)，用于控制状态机的运行（如启停、复位）
    int modeOfOperation;    // 操作模式 (8位)，表示当前的控制模式 (如位置模式、速度模式等)
    int torqueOffset;       // 扭矩偏置 (16位)，用于补偿扭矩的偏移量
};
struct RPDO_CST_Mode
{
    int targetTorque;       // 目标扭矩 (16位)，控制设备输出的力矩大小
    int controlWord;        // 控制字 (16位)，用于控制设备的状态机，执行启停、复位等命令
    int modeOfOperation;    // 操作模式 (8位)，决定当前控制模式（如位置模式、速度模式、扭矩模式等）
};
