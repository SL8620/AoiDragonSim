/*** 
 * @Author: SL8620
 * @Date: 2024-10-29 19:59:47
 * @LastEditTime: 2024-10-30 10:45:31
 * @LastEditors: SL8620
 * @Description: 
 * @FilePath: \AzureLoongSim\RoboDataBus\Pdo_Struct.h
 * @��������Ԥ���İ�Ȩ����������ǩ�������е�
 */

#pragma once

//TPDO���䷽�������ˣ�Elmo��->���ƶˣ�STM32��
struct TPDO_STD_Mode
{
    int actualPosition;     // ʵ��λ�� (32λ)���豸��ǰ��ʵ������λ��
    int actualVelocity;     // ʵ���ٶ� (32λ)���豸��ǰ��ʵ�������ٶ�
    int actualTorque;       // ʵ��Ť�� (16λ)���豸��ǰ�����ʵ��Ť��
    int statusword;         // ״̬�� (16λ)����ʾ�豸�ĵ�ǰ״̬���������С��������Ϣ
    int actualCurrent;      // ʵ�ʵ��� (16λ)�������ǰ��ʵ�ʵ���
};
struct TPDO_CST_Mode
{
    int targetPosition;     // Ŀ��λ�� (32λ)�����ڿ��Ƶ�����豸��λ��
    int targetVelocity;     // Ŀ���ٶ� (32λ)�����ڿ��Ƶ�����豸���ٶ�
    int targetTorque;       // Ŀ��Ť�� (16λ)�����ڿ����豸��Ť�����
    int maxTorque;          // ���Ť��(16λ)����ʾ��������Ť������
    int controlWord;        // ������ (16λ)�����ڿ���״̬�������У�����ͣ����λ��
    int modeOfOperation;    // ����ģʽ (8λ)����ʾ��ǰ�Ŀ���ģʽ (��λ��ģʽ���ٶ�ģʽ��)
    int torqueOffset;       // Ť��ƫ�� (16λ)�����ڲ���Ť�ص�ƫ����
};
//RPDO���䷽�򣺿��ƶˣ�STM32��->�����ˣ�Elmo��
struct RPDO_STD_Mode
{
    int targetPosition;     // Ŀ��λ�� (32λ)�����ڿ��Ƶ�����豸��λ��
    int targetVelocity;     // Ŀ���ٶ� (32λ)�����ڿ��Ƶ�����豸���ٶ�
    int targetTorque;       // Ŀ��Ť�� (16λ)�����ڿ����豸��Ť�����
    int maxTorque;          // ���Ť��(16λ)����ʾ��������Ť������
    int controlWord;        // ������ (16λ)�����ڿ���״̬�������У�����ͣ����λ��
    int modeOfOperation;    // ����ģʽ (8λ)����ʾ��ǰ�Ŀ���ģʽ (��λ��ģʽ���ٶ�ģʽ��)
    int torqueOffset;       // Ť��ƫ�� (16λ)�����ڲ���Ť�ص�ƫ����
};
struct RPDO_CST_Mode
{
    int targetTorque;       // Ŀ��Ť�� (16λ)�������豸��������ش�С
    int controlWord;        // ������ (16λ)�����ڿ����豸��״̬����ִ����ͣ����λ������
    int modeOfOperation;    // ����ģʽ (8λ)��������ǰ����ģʽ����λ��ģʽ���ٶ�ģʽ��Ť��ģʽ�ȣ�
};
