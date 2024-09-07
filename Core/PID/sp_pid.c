

#include <stdlib.h>
#include "sp_pid.h"

// include the header for PID data structure definition

void sp_pid_init(SP_PID *v)
{
    //���³�ʼ��ʱ��Խ�Ծ�������

    v->ErrorAbsMax = v->Ref * 0.30f;
    v->ErrorAbsMid = v->Ref * 0.15f;
    v->ErrorAbsMin = v->Ref * 0.05f;

//    v->ErrorAbsMax = v->Ref *0.15f;
//    v->ErrorAbsMid = v->Ref *0.08f;
//    v->ErrorAbsMin = v->Ref *0.03f;
}



void sp_pid_calc(SP_PID *v)
{

    float Delta_Err;                  //e(k)-e(k-1)
    float Last_Delta_Err;             //e(k-1)-e(k-2)
    float uk;                         //���ε������ֵ

    v->Err = v->Ref - v->Fdb;
    Delta_Err = v->Err - v->Err_1;
    Last_Delta_Err = v->Err_1 - v->Err_2;

    if (fabs(v->Err) >= v->ErrorAbsMax)
    {
        /*ִ�й���1*/
        if (v->Err > 0)
        {
            uk = v->OutMax;
            uk = v->Ref / 12 * 4000;
        }
        if (v->Err < 0)
        {
            uk = v->OutMin;
            uk = v->Ref / 12 * 4000;
        }
    }

    if ((v->Err * Delta_Err > 0) || (Delta_Err == 0))
    {
        /*ִ�й���2*/
        if (fabs(v->Err) >= v->ErrorAbsMid)
        {
            uk = v->Out + 1.5f * (v->Kp * Delta_Err + v->Ki * v->Err + v->Kd * (Delta_Err - Last_Delta_Err));
        }
        else
        {
            uk = v->Out + 0.3f * (v->Kp * Delta_Err + v->Ki * v->Err + v->Kd * (Delta_Err - Last_Delta_Err));
        }
    }

    if (((v->Err * Delta_Err < 0) && (Delta_Err * Last_Delta_Err > 0)) || (v->Err == 0))
    {
        /*ִ�й���3*/
        uk = v->Out;
    }

    if ((v->Err * Delta_Err < 0) && (Delta_Err * Last_Delta_Err < 0))
    {
        /*ִ�й���4*/
        if (fabs(v->Err) >= v->ErrorAbsMid)
        {
            uk = v->Out + 1.5f * v->Kp * v->Err;
        }
        else
        {
            uk = v->Out + 0.4f * v->Kp * v->Err;
        }
    }

    if ((fabs(v->Err) <= v->ErrorAbsMin) && (fabs(v->Err) > 0))
    {
        /*ִ�й���5*/
        uk = v->Out + 0.5f * v->Kp * Delta_Err + 0.3f * v->Ki * v->Err;
    }

    v->OutPreSat = uk;

    // Saturate the output
    if (v->OutPreSat > v->OutMax)
        v->Out =  v->OutMax;
    else if (v->OutPreSat < v->OutMin)
        v->Out =  v->OutMin;
    else
        v->Out = v->OutPreSat;

    v->Err_2 = v->Err_1;
    v->Err_1 = v->Err;


}


