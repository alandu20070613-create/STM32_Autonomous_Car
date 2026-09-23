#ifndef __MPU6050_H
#define __MPU6050_H
#include "mpuiic.h"
#include "stm32f1xx_hal.h"

// MPU6050 AD0 pin control, CubeMX alias: MPU_AD0
#define MPU_AD0_CTRL		HAL_GPIO_WritePin(MPU_AD0_GPIO_Port, MPU_AD0_Pin, GPIO_PIN_RESET)
#define MPU_AD0_CTRL_HIGH	HAL_GPIO_WritePin(MPU_AD0_GPIO_Port, MPU_AD0_Pin, GPIO_PIN_SET)

// MPU6050 Register Address Definition
#define MPU_SELF_TESTX_REG		0X0D	// Self-test register X
#define MPU_SELF_TESTY_REG		0X0E	// Self-test register Y
#define MPU_SELF_TESTZ_REG		0X0F	// Self-test register Z
#define MPU_SELF_TESTA_REG		0X10	// Self-test register Accelerometer
#define MPU_SAMPLE_RATE_REG	0X19	// Sample rate divider register
#define MPU_CFG_REG				0X1A	// Configuration register (LPF)
#define MPU_GYRO_CFG_REG		0X1B	// Gyroscope configuration register
#define MPU_ACCEL_CFG_REG		0X1C	// Accelerometer configuration register
#define MPU_MOTION_DET_REG		0X1F	// Motion detection threshold register
#define MPU_FIFO_EN_REG			0X23	// FIFO enable register
#define MPU_I2CMST_CTRL_REG	0X24	// I2C master control register
#define MPU_I2CSLV0_ADDR_REG	0X25	// I2C slave 0 address register
#define MPU_I2CSLV0_REG			0X26	// I2C slave 0 data register
#define MPU_I2CSLV0_CTRL_REG	0X27	// I2C slave 0 control register
#define MPU_I2CSLV1_ADDR_REG	0X28	// I2C slave 1 address register
#define MPU_I2CSLV1_REG			0X29	// I2C slave 1 data register
#define MPU_I2CSLV1_CTRL_REG	0X2A	// I2C slave 1 control register
#define MPU_I2CSLV2_ADDR_REG	0X2B	// I2C slave 2 address register
#define MPU_I2CSLV2_REG			0X2C	// I2C slave 2 data register
#define MPU_I2CSLV2_CTRL_REG	0X2D	// I2C slave 2 control register
#define MPU_I2CSLV3_ADDR_REG	0X2E	// I2C slave 3 address register
#define MPU_I2CSLV3_REG			0X2F	// I2C slave 3 data register
#define MPU_I2CSLV3_CTRL_REG	0X30	// I2C slave 3 control register
#define MPU_I2CSLV4_ADDR_REG	0X31	// I2C slave 4 address register
#define MPU_I2CSLV4_REG			0X32	// I2C slave 4 data register
#define MPU_I2CSLV4_DO_REG		0X33	// I2C slave 4 data-out register
#define MPU_I2CSLV4_CTRL_REG	0X34	// I2C slave 4 control register
#define MPU_I2CSLV4_DI_REG		0X35	// I2C slave 4 data-in register

#define MPU_I2CMST_STA_REG		0X36	// I2C master status register
#define MPU_INTBP_CFG_REG		0X37	// Interrupt/bypass configuration register
#define MPU_INT_EN_REG			0X38	// Interrupt enable register
#define MPU_INT_STA_REG			0X3A	// Interrupt status register

#define MPU_ACCEL_XOUTH_REG	0X3B	// Accelerometer X high byte
#define MPU_ACCEL_XOUTL_REG	0X3C	// Accelerometer X low byte
#define MPU_ACCEL_YOUTH_REG	0X3D	// Accelerometer Y high byte
#define MPU_ACCEL_YOUTL_REG	0X3E	// Accelerometer Y low byte
#define MPU_ACCEL_ZOUTH_REG	0X3F	// Accelerometer Z high byte
#define MPU_ACCEL_ZOUTL_REG	0X40	// Accelerometer Z low byte

#define MPU_TEMP_OUTH_REG		0X41	// Temperature high byte
#define MPU_TEMP_OUTL_REG		0X42	// Temperature low byte

#define MPU_GYRO_XOUTH_REG		0X43	// Gyroscope X high byte
#define MPU_GYRO_XOUTL_REG		0X44	// Gyroscope X low byte
#define MPU_GYRO_YOUTH_REG		0X45	// Gyroscope Y high byte
#define MPU_GYRO_YOUTL_REG		0X46	// Gyroscope Y low byte
#define MPU_GYRO_ZOUTH_REG		0X47	// Gyroscope Z high byte
#define MPU_GYRO_ZOUTL_REG		0X48	// Gyroscope Z low byte

#define MPU_I2CSLV0_DO_REG		0X63	// I2C slave0 data out
#define MPU_I2CSLV1_DO_REG		0X64	// I2C slave1 data out
#define MPU_I2CSLV2_DO_REG		0X65	// I2C slave2 data out
#define MPU_I2CSLV3_DO_REG		0X66	// I2C slave3 data out

#define MPU_I2CMST_DELAY_REG	0X67	// I2C master delay register
#define MPU_SIGPATH_RST_REG		0X68	// Signal path reset register
#define MPU_MDETECT_CTRL_REG	0X69	// Motion detect control register
#define MPU_USER_CTRL_REG		0X6A	// User control register
#define MPU_PWR_MGMT1_REG		0X6B	// Power management register 1
#define MPU_PWR_MGMT2_REG		0X6C	// Power management register 2
#define MPU_FIFO_CNTH_REG		0X72	// FIFO count high byte
#define MPU_FIFO_CNTL_REG		0X73	// FIFO count low byte
#define MPU_FIFO_RW_REG			0X74	// FIFO read/write register
#define MPU_DEVICE_ID_REG		0X75	// Device ID register

// If AD0 pin connected to GND, IIC address = 0x68
// If AD0 pin connected to VCC, IIC address = 0x69
#define MPU_ADDR				0X68

// Function prototypes
uint8_t MPU_Init(void);
uint8_t MPU_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf);	// Continuous IIC write multiple bytes
uint8_t MPU_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf);	// Continuous IIC read multiple bytes
uint8_t MPU_Write_Byte(uint8_t reg,uint8_t data);							// Write single byte via IIC
uint8_t MPU_Read_Byte(uint8_t reg);											// Read single byte via IIC

uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr);
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr);
uint8_t MPU_Set_LPF(uint16_t lpf);
uint8_t MPU_Set_Rate(uint16_t rate);
uint8_t MPU_Set_Fifo(uint8_t sens);

short MPU_Get_Temperature(void);
uint8_t MPU_Get_Gyroscope(short *gx,short *gy,short *gz);
uint8_t MPU_Get_Accelerometer(short *ax,short *ay,short *az);

#endif
