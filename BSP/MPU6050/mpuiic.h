#ifndef __MPUIIC_H
#define __MPUIIC_H

#include "stm32f1xx_hal.h"

#define SCL_6050_GPIO_Port GPIOB
#define SCL_6050_Pin GPIO_PIN_8

#define SDA_6050_GPIO_Port GPIOB
#define SDA_6050_Pin GPIO_PIN_9

#define MPU_AD0_GPIO_Port GPIOA
#define MPU_AD0_Pin GPIO_PIN_15


// Switch SDA pin mode: Input / Output
#define MPU_SDA_IN()        HAL_GPIO_Init_SDA_INPUT()
#define MPU_SDA_OUT()       HAL_GPIO_Init_SDA_OUTPUT()

// IIC level control macros, pin aliases defined in CubeMX: SCL_6050 / SDA_6050
#define MPU_IIC_SCL_HIGH    HAL_GPIO_WritePin(SCL_6050_GPIO_Port, SCL_6050_Pin, GPIO_PIN_SET)
#define MPU_IIC_SCL_LOW     HAL_GPIO_WritePin(SCL_6050_GPIO_Port, SCL_6050_Pin, GPIO_PIN_RESET)
#define MPU_IIC_SDA_HIGH    HAL_GPIO_WritePin(SDA_6050_GPIO_Port, SDA_6050_Pin, GPIO_PIN_SET)
#define MPU_IIC_SDA_LOW     HAL_GPIO_WritePin(SDA_6050_GPIO_Port, SDA_6050_Pin, GPIO_PIN_RESET)

// Read SDA pin input level
#define MPU_READ_SDA        HAL_GPIO_ReadPin(SDA_6050_GPIO_Port, SDA_6050_Pin)

// Function declarations for software IIC
void MPU_IIC_Delay(void);                             // IIC basic delay (2us)
void MPU_IIC_Init(void);                              // Initialize IIC GPIO pins
void MPU_IIC_Start(void);                             // Generate IIC start signal
void MPU_IIC_Stop(void);                              // Generate IIC stop signal
void MPU_IIC_Send_Byte(uint8_t txd);                  // Send one byte via IIC
uint8_t MPU_IIC_Read_Byte(unsigned char ack);         // Read one byte via IIC, ack=1 send ACK, ack=0 send NACK
uint8_t MPU_IIC_Wait_Ack(void);                       // Wait for slave ACK signal
void MPU_IIC_Ack(void);                               // Master send ACK
void MPU_IIC_NAck(void);                              // Master send NACK

void MPU_IIC_Write_One_Byte(uint8_t daddr,uint8_t addr,uint8_t data);
uint8_t MPU_IIC_Read_One_Byte(uint8_t daddr,uint8_t addr);

// Private helper functions to switch SDA mode, implemented in .c
void HAL_GPIO_Init_SDA_INPUT(void);
void HAL_GPIO_Init_SDA_OUTPUT(void);

#endif
