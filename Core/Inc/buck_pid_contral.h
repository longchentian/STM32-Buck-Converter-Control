#ifndef __BUCK_PID_CONTRAL_H__
#define __BUCK_PID_CONTRAL_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/	
#include "main.h"

	
//#include "hrtim.h"
//#include "tim.h"
//#include "usart.h"
//#include "adc.h"
#include "aw_pid.h"
#include "sp_pid.h"
#include "eh_pid.h"
#include "fuzzy_pid.h"

#define  set_buck_enable()	HAL_GPIO_WritePin(ENA_GPIO_Port, ENA_Pin, GPIO_PIN_SET)
#define  set_buck_disable()	HAL_GPIO_WritePin(ENA_GPIO_Port, ENA_Pin, GPIO_PIN_RESET)

#define PID3


#if defined(PID1)  
	extern	AW_PID pid;
#elif defined(PID2)  
	extern	SP_PID pid ;
#elif defined(PID3)	
	extern	EH_PID pid;	
	
#endif  
	
extern int rule_base[21][qf_default];
extern int mf_params[4 * qf_default];
extern float fuzzy_pid_params[1][pid_params_count] ;
	
void set_p_i_d(float p, float i, float d);
void set_pid_target(float temp_val);
float get_pid_target(void);

	
	

	
#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
