#ifndef INCLUDE_SP_PID_H_
#define INCLUDE_SP_PID_H_
#include "stm32h7xx.h"


typedef  struct
{
    float  Ref;           // ���룺�ο�����  Input: Reference input
    float  Fdb;           // ���룺��������  Input: Feedback input
    float  Err;            //ƫ��
    float  Err_1;          // ����������ź�e(k-1)   Variable: Error
    float  Err_2;          // ����������ź�e(k-2)   Variable: Error
    float  Ts;            // ��������


    float  Kp;            // ��������������  Parameter: Proportional gain
    float  Ki;            // �������������� Parameter: Integral gain
    float  Kd;            // ������΢������Parameter: Derivative gain


    float ErrorAbsMax;            /*ƫ�����ֵ���ֵ*/
    float ErrorAbsMid;            /*ƫ�����ֵ��λֵ*/
    float ErrorAbsMin;            /*ƫ�����ֵ��Сֵ*/

    float  OutPreSat;     // �������������  Variable: Pre-saturated output
    float  OutMax;        // ������������  Parameter: Maximum output
    float  OutMin;        // ��������С���  Parameter: Minimum output
    float  Out;           // �����SP_PID���  Output: SP_PID output

    void (*init)();          // Pointer to the init funcion
    void (*calc)();         // ���㺯��ָ��  Pointer to calculation function
} SP_PID;
typedef   SP_PID  *SP_PID_handle;
/*-----------------------------------------------------------------------------
  Ĭ�ϳ�ʼ��   Default initalizer for the AW_PID object.
-----------------------------------------------------------------------------*/
#define  SP_PID_DEFAULTS { 0, 0, 0, 0, 0, 0,\
                           0, 0, 0, \
                           0, 0, 0,\
                           0, 0, 0, 0, \
                           (void (*)(unsigned long)) sp_pid_init,\
                           (void (*)(unsigned long)) sp_pid_calc }
/*------------------------------------------------------------------------------
  ����ԭ��   Prototypes for the functions in  <SP_PID.c>
------------------------------------------------------------------------------*/

void sp_pid_init(SP_PID_handle);
void sp_pid_calc(SP_PID_handle);


#endif /* INCLUDE_SP_PID_H_ */



