/*
 * AW_PID.h
 *
 *  Created on: 2021��9��29��
 *      Author: longchentian
 */

#ifndef INCLUDE_AW_PID_H_
#define INCLUDE_AW_PID_H_
#include "stm32h7xx.h"

typedef  struct
{
    float  Ref;           // ���룺�ο�����  Input: Reference input
    float  Fdb;           // ���룺��������  Input: Feedback input
    float  Err;           // ����������ź�e(k)
    float  Err_1;          // ����������ź�e(k-1)   Variable: Error
    float  umax;          // �������������ı����޷�ֵ
    float  Ts;            // ��������

    float  kw;            // �����������ͷ�����0 ���ޣ���1���������֣���2���������ƣ�
    float  Kp;            // ��������������  Parameter: Proportional gain
    float  Ki;            // �������������� Parameter: Integral gain
    float  Kc;            // ������������������ Parameter: Integral correction gain
    float  Kd;            // ������΢������Parameter: Derivative gain

    float  Up;            // �������������  Variable: Proportional output
    float  Ui;            // �������������  Variable: Integral output
    float  Ud;            // ������΢�����  Variable: Derivative output
    float  Ud1;           // ������΢�����(k-1)  Variable: Derivative output
    float  N;             // ��������ͨ�˲�����

    float  OutPreSat;     // �������������  Variable: Pre-saturated output
    float  OutMax;        // ������������  Parameter: Maximum output
    float  OutMin;        // ��������С���  Parameter: Minimum output
    float  Out;           // �����AW_PID���  Output: AW_PID output
    float  SatErr;        // ���������Ͳ�ֵ Variable: Saturated difference
    void (*calc)();         // ���㺯��ָ��  Pointer to calculation function
} AW_PID;
typedef   AW_PID  *AW_PID_handle;
/*-----------------------------------------------------------------------------
  Ĭ�ϳ�ʼ��   Default initalizer for the AW_PID object.
-----------------------------------------------------------------------------*/
#define  AW_PID_DEFAULTS { 0, 0, 0, 0, 0, 0, \
                           0, 0, 0, 0, 0, \
                           0, 0, 0, 0, 10, \
                           0, 0, 0, 0, 0, \
                    (void (*)(unsigned long)) aw_pid_calc }
/*------------------------------------------------------------------------------
  ����ԭ��   Prototypes for the functions in  <AW_PID.c>
------------------------------------------------------------------------------*/
void aw_pid_calc(AW_PID_handle);


#endif /* INCLUDE_AW_PID_H_ */
