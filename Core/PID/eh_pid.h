/*
 * eh_pid.h
 *
 *  Created on: 2021��10��10��
 *      Author: longchentian
 */

#ifndef EH_PID_H_
#define EH_PID_H_

#include "stm32h7xx.h"

typedef  struct
{
    float  Ref;           // ���룺�ο�����  Input: Reference input
    float  Fdb;           // ���룺��������  Input: Feedback input��
    float  Err;            //����������ź�e(k)
    float  Err_1;          // ����������ź�e(k-1)   Variable: Error
    float  Err_2;          // ����������ź�e(k-2)   Variable: Error
    float  umax;          // �������������ı����޷�ֵ
    float  Ts;            // ��������

    float  Kp;            // ��������������  Parameter: Proportional gain
    float  Ki;            // �������������� Parameter: Integral gain
    float  Kd;            // ������΢������Parameter: Derivative gain


    float  OutPreSat;     // �������������  Variable: Pre-saturated output
    float  OutMax;        // ������������  Parameter: Maximum output
    float  OutMin;        // ��������С���  Parameter: Minimum output
    float  Out;           // �����SP_PID���  Output: SP_PID output


    void (*calc)();         // ���㺯��ָ��  Pointer to calculation function
} EH_PID;
typedef   EH_PID  *EH_PID_handle;


/*-----------------------------------------------------------------------------
  Ĭ�ϳ�ʼ��   Default initalizer for the AW_PID object.
-----------------------------------------------------------------------------*/
#define  EH_PID_DEFAULTS { 0, 0, 0, 0, 0, 0, 0,\
                           0, 0, 0, \
                           0, 0, 0, 0, \
                    (void (*)(unsigned long)) eh_pid_calc }

void eh_pid_calc(EH_PID_handle);

#endif /* EH_PID_H_ */
