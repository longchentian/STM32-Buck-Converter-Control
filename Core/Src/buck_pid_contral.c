#include "buck_pid_contral.h"
#include "aw_pid.h"
#include "sp_pid.h"
#include "eh_pid.h"




#if defined(PID1)  
    AW_PID pid = AW_PID_DEFAULTS;
#elif defined(PID2)  
    SP_PID pid = SP_PID_DEFAULTS;
#elif defined(PID3)	
	EH_PID pid = EH_PID_DEFAULTS;

#endif  



int rule_base[][qf_default] = {
		//delta kp rule base
		{PB, PB, PM, PM, PS, ZO, ZO},
		{PB, PB, PM, PS, PS, ZO, NS},
		{PM, PM, PM, PS, ZO, NS, NS},
		{PM, PM, PS, ZO, NS, NM, NM},
		{PS, PS, ZO, NS, NS, NM, NM},
		{PS, ZO, NS, NM, NM, NM, NB},
		{ZO, ZO, NM, NM, NM, NB, NB},
		//delta ki rule base
		{NB, NB, NM, NM, NS, ZO, ZO},
		{NB, NB, NM, NS, NS, ZO, ZO},
		{NB, NM, NS, NS, ZO, PS, PS},
		{NM, NM, NS, ZO, PS, PM, PM},
		{NM, NS, ZO, PS, PS, PM, PB},
		{ZO, ZO, PS, PS, PM, PB, PB},
		{ZO, ZO, PS, PM, PM, PB, PB},
		//delta kd rule base
		{PS, NS, NB, NB, NB, NM, PS},
		{PS, NS, NB, NM, NM, NS, ZO},
		{ZO, NS, NM, NM, NS, NS, ZO},
		{ZO, NS, NS, NS, NS, NS, ZO},
		{ZO, ZO, ZO, ZO, ZO, ZO, ZO},
		{PB, PS, PS, PS, PS, PS, PB},
		{PB, PM, PM, PM, PS, PS, PB}};

// Default parameters of membership function
int mf_params[4 * qf_default] = {-3, -3, -2, 0,
								 -3, -2, -1, 0,
								 -2, -1,  0, 0,
								 -1,  0,  1, 0,
								  0,  1,  2, 0,
								  1,  2,  3, 0,
								  2,  3,  3, 0};

float fuzzy_pid_params[1][pid_params_count] = {{10.4597502f,  5.0053997f,    8.59500027,    0, 0, 0, 1}};






void set_pid_target(float temp_val)
{
	float temp;
	temp =(float) temp_val/1000;
	pid.Ref = temp_val;    // 设置当前的目标值
}

float get_pid_target(void)
{
  return pid.Ref;    // 设置当前的目标值
}


void set_p_i_d(float p, float i, float d)
{
  pid.Kp = p;    // 设置比例系数 P
  pid.Ki = i;    // 设置积分系数 I
  pid.Kd = d;    // 设置微分系数 D
}

