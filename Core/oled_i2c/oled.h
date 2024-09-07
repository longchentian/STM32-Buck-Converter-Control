
/****************************************************************************************************
//=========================================电源接线================================================//
//      5V  接DC 5V电源
//     GND  接地
//======================================OLED屏数据线接线==========================================//
//本模块数据总线类型为IIC
//     SCL  接PB13    // IIC时钟信号
//     SDA  接PB14    // IIC数据信号
//======================================OLED屏控制线接线==========================================//
//本模块数据总线类型为IIC，不需要接控制信号线    
//=========================================触摸屏接线=========================================//
//本模块本身不带触摸，不需要接触摸屏线
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

#define OLED_SCL_Clr()  HAL_GPIO_WritePin(SCL_GPIO_Port,SCL_Pin, GPIO_PIN_RESET)//SDA IIC接口的时钟信号
#define OLED_SCL_Set() 	HAL_GPIO_WritePin(SCL_GPIO_Port,SCL_Pin, GPIO_PIN_SET)

#define OLED_SDA_Clr() 	HAL_GPIO_WritePin(SDA_GPIO_Port,SDA_Pin, GPIO_PIN_RESET)//SCL IIC接口的数据信号
#define OLED_SDA_Set()  HAL_GPIO_WritePin(SDA_GPIO_Port,SDA_Pin, GPIO_PIN_SET)

#define IIC_DELAY_Value      3000 //该值根据自身CPU性能更改，h750用3000没问题

/*含有RES引脚的需要重写*/
#define OLED_RES_Clr() NULL
#define OLED_RES_Set() NULL


//-----------------OLED IIC端口定义----------------  					   

#define OLED_I2C    1

#define OLED_SDA_Pin 			GPIO_PIN_6
#define OLED_SDA_GPIO_Port 		GPIOB
#define OLED_SDA_CLK_ENABLE()    	__HAL_RCC_GPIOB_CLK_ENABLE()
#define OLED_SCL_Pin 			GPIO_PIN_7
#define OLED_SCL_GPIO_Port 		GPIOB
#define OLED_SCL_CLK_ENABLE()    	__HAL_RCC_GPIOB_CLK_ENABLE()




 		     
#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据

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
	 



