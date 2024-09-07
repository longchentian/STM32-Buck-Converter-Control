
/****************************************************************************************************
//=========================================��Դ����================================================//
//      5V  ��DC 5V��Դ
//     GND  �ӵ�
//======================================OLED�������߽���==========================================//
//��ģ��������������ΪIIC
//     SCL  ��PB13    // IICʱ���ź�
//     SDA  ��PB14    // IIC�����ź�
//======================================OLED�������߽���==========================================//
//��ģ��������������ΪIIC������Ҫ�ӿ����ź���    
//=========================================����������=========================================//
//��ģ�鱾��������������Ҫ�Ӵ�������
//============================================================================================//
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, QD electronic SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
**************************************************************************************************/	


#ifndef __OLED_H
#define __OLED_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdlib.h"
#include "stm32h7xx_hal.h"
//#include "i2c.h"
/* USER CODE BEGIN Private defines */


#define SDA_Pin 			GPIO_PIN_13
#define SDA_GPIO_Port 		GPIOB
#define SDA_CLK_ENABLE()    __HAL_RCC_GPIOB_CLK_ENABLE()
#define SCL_Pin 			GPIO_PIN_15
#define SCL_GPIO_Port 		GPIOB
#define SCL_CLK_ENABLE()    __HAL_RCC_GPIOB_CLK_ENABLE()

#define OLED_SCL_Clr()  HAL_GPIO_WritePin(SCL_GPIO_Port,SCL_Pin, GPIO_PIN_RESET)//SDA IIC�ӿڵ�ʱ���ź�
#define OLED_SCL_Set() 	HAL_GPIO_WritePin(SCL_GPIO_Port,SCL_Pin, GPIO_PIN_SET)

#define OLED_SDA_Clr() 	HAL_GPIO_WritePin(SDA_GPIO_Port,SDA_Pin, GPIO_PIN_RESET)//SCL IIC�ӿڵ������ź�
#define OLED_SDA_Set()  HAL_GPIO_WritePin(SDA_GPIO_Port,SDA_Pin, GPIO_PIN_SET)

#define IIC_DELAY_Value      3000 //��ֵ��������CPU���ܸ��ģ�h750��3000û����

/*����RES���ŵ���Ҫ��д*/
#define OLED_RES_Clr() NULL
#define OLED_RES_Set() NULL


//-----------------OLED IIC�˿ڶ���----------------  					   

#define OLED_I2C    1

#define OLED_SDA_Pin 			GPIO_PIN_6
#define OLED_SDA_GPIO_Port 		GPIOB
#define OLED_SDA_CLK_ENABLE()    	__HAL_RCC_GPIOB_CLK_ENABLE()
#define OLED_SCL_Pin 			GPIO_PIN_7
#define OLED_SCL_GPIO_Port 		GPIOB
#define OLED_SCL_CLK_ENABLE()    	__HAL_RCC_GPIOB_CLK_ENABLE()




 		     
#define OLED_CMD  0	//д����
#define OLED_DATA 1	//д����

#define IIC_SLAVE_ADDR 0x78  //IIC slave device address


/* USER CODE END Private defines */

#ifndef OLED_I2C
#define OLED_IIC_HANDLE hi2c1
#else
	#if OLED_I2C==1
	void I2C_Start(void);
	void I2C_Stop(void);
	void I2C_WaitAck(void);
	void Send_Byte(uint8_t dat);
#endif
	
#endif






void OLED_ClearPoint(uint8_t x,uint8_t y);
void OLED_ColorTurn(uint8_t i);
void OLED_DisplayTurn(uint8_t i);
void OLED_WR_Byte(uint8_t dat,uint8_t mode);
void OLED_DisPlay_On(void);
void OLED_DisPlay_Off(void);
void OLED_Refresh(void);
void OLED_Clear(void);
void OLED_DrawPoint(uint8_t x,uint8_t y,uint8_t t);
void OLED_DrawLine(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2,uint8_t mode);
void OLED_DrawCircle(uint8_t x,uint8_t y,uint8_t r);
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t size1,uint8_t mode);
void OLED_ShowChar6x8(uint8_t x,uint8_t y,uint8_t chr,uint8_t mode);
void OLED_ShowString(uint8_t x,uint8_t y,uint8_t *chr,uint8_t size1,uint8_t mode);
void OLED_ShowNum(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size1,uint8_t mode);
void OLED_ShowChinese(uint8_t x,uint8_t y,uint8_t num,uint8_t size1,uint8_t mode);
void OLED_ScrollDisplay(uint8_t num,uint8_t space,uint8_t mode);
void OLED_ShowPicture(uint8_t x,uint8_t y,uint8_t sizex,uint8_t sizey,uint8_t BMP[],uint8_t mode);
void OLED_Init(void);

void OLED_IIC_Init(void);

#ifdef __cplusplus
}
#endif
#endif  
	 



