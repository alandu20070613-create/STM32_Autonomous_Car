#include "mpu6050.h"
#include "delay.h"

/**
 * @brief  Initialize MPU6050 sensor
 * @retval 0: init success; 1: init fail (wrong device ID)
 */
uint8_t MPU_Init(void)
{
	uint8_t res;

	// Enable GPIOA clock
	__HAL_RCC_GPIOA_CLK_ENABLE();

	// Initialize PA15 (AD0 pin) as push-pull output
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.Pin = MPU_AD0_Pin;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(MPU_AD0_GPIO_Port, &GPIO_InitStructure);

	// Disable JTAG function on PA15, release PA15 as normal GPIO
	__HAL_AFIO_REMAP_SWJ_NOJTAG();

	MPU_AD0_CTRL;	// AD0 low, IIC address = 0x68

	MPU_IIC_Init();
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X80);	// Reset MPU6050
	HAL_Delay(100);
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X00);	// Wake up MPU6050
	MPU_Set_Gyro_Fsr(3);						// Gyro full scale range: ±2000dps
	MPU_Set_Accel_Fsr(0);						// Accel full scale range: ±2g
	MPU_Set_Rate(50);							// Sample rate = 50Hz
	MPU_Write_Byte(MPU_INT_EN_REG,0X00);		// Disable all interrupts
	MPU_Write_Byte(MPU_USER_CTRL_REG,0X00);	// Close I2C slave mode
	MPU_Write_Byte(MPU_FIFO_EN_REG,0X00);		// Disable FIFO
	MPU_Write_Byte(MPU_INTBP_CFG_REG,0X80);	// Interrupt active high, open drain

	res=MPU_Read_Byte(MPU_DEVICE_ID_REG);
	if(res==MPU_ADDR)
	{
		MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X01);	// Clock source: PLL X gyro
		MPU_Write_Byte(MPU_PWR_MGMT2_REG,0X00);	// No sleep for accel/gyro
		MPU_Set_Rate(50);
 	}else return 1;
	return 0;
}

/**
 * @brief  Configure gyroscope full-scale range
 * @param  fsr: 0=±250dps,1=±500dps,2=±1000dps,3=±2000dps
 * @retval 0: write success
 */
uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU_GYRO_CFG_REG,fsr<<3);
}

/**
 * @brief  Configure accelerometer full-scale range
 * @param  fsr: 0=±2g,1=±4g,2=±8g,3=±16g
 * @retval 0: write success
 */
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU_ACCEL_CFG_REG,fsr<<3);
}

/**
 * @brief  Set hardware low-pass filter cutoff frequency
 * @param  lpf: target cutoff frequency(Hz)
 * @retval 0: write success
 */
uint8_t MPU_Set_LPF(uint16_t lpf)
{
	uint8_t data=0;

	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6;
	return MPU_Write_Byte(MPU_CFG_REG,data);
}

/**
 * @brief  Set MPU6050 sample output rate (base clock 1kHz)
 * @param  rate: target sample rate 4~1000Hz, default 50Hz
 * @retval 0: config success
 */
uint8_t MPU_Set_Rate(uint16_t rate)
{
	uint8_t data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(MPU_SAMPLE_RATE_REG,data);
 	return MPU_Set_LPF(rate/2);	// Automatically set LPF to half sample frequency
}

/**
 * @brief  Read temperature raw value, return temperature*100
 * @retval temperature multiplied by 100
 */
short MPU_Get_Temperature(void)
{
   uint8_t buf[2];
   short raw;
	 float temp;

	 MPU_Read_Len(MPU_ADDR,MPU_TEMP_OUTH_REG,2,buf);
   raw=((uint16_t)buf[0]<<8)|buf[1];
   temp=36.53+((double)raw)/340;
   return temp*100;
}

/**
 * @brief  Read raw gyroscope X/Y/Z data
 * @param  gx: pointer store X raw value
 * @param  gy: pointer store Y raw value
 * @param  gz: pointer store Z raw value
 * @retval 0: read success; other: fail
 */
uint8_t MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
{
  uint8_t buf[6],res;

	res=MPU_Read_Len(MPU_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
		*gx=((uint16_t)buf[0]<<8)|buf[1];
		*gy=((uint16_t)buf[2]<<8)|buf[3];
		*gz=((uint16_t)buf[4]<<8)|buf[5];
	}
  return res;
}

/**
 * @brief  Read raw accelerometer X/Y/Z data
 * @param  ax: pointer store X raw value
 * @param  ay: pointer store Y raw value
 * @param  az: pointer store Z raw value
 * @retval 0: read success; other: fail
 */
uint8_t MPU_Get_Accelerometer(short *ax,short *ay,short *az)
{
    uint8_t buf[6],res;
	res=MPU_Read_Len(MPU_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
	if(res==0)
	{
		*ax=((uint16_t)buf[0]<<8)|buf[1];
		*ay=((uint16_t)buf[2]<<8)|buf[3];
		*az=((uint16_t)buf[4]<<8)|buf[5];
	}
    return res;
}

/**
 * @brief  Continuous multi-byte IIC write
 * @param  addr: device IIC address
 * @param  reg: start register address
 * @param  len: total bytes to write
 * @param  buf: data buffer pointer
 * @retval 0: write success; 1: fail
 */
uint8_t MPU_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
	uint8_t i;

	MPU_IIC_Start();
	MPU_IIC_Send_Byte((addr<<1)|0);      // Device address + write flag (0=write,1=read)
	if(MPU_IIC_Wait_Ack())               // Wait slave ACK
	{
		MPU_IIC_Stop();
		return 1;
	}
    MPU_IIC_Send_Byte(reg);              // Send target register address
    MPU_IIC_Wait_Ack();
	for(i=0;i<len;i++)
	{
		MPU_IIC_Send_Byte(buf[i]);
		if(MPU_IIC_Wait_Ack())
		{
			MPU_IIC_Stop();
			return 1;
		}
	}
    MPU_IIC_Stop();
	return 0;
}

/**
 * @brief  Continuous multi-byte IIC read
 * @param  addr: device IIC address
 * @param  reg: start register address
 * @param  len: total bytes to read
 * @param  buf: receive data buffer pointer
 * @retval 0: read success; 1: fail
 */
uint8_t MPU_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
		MPU_IIC_Start();
		MPU_IIC_Send_Byte((addr<<1)|0);		// Send address + write for register selection
		if(MPU_IIC_Wait_Ack())
		{
			MPU_IIC_Stop();
			return 1;
		}
    MPU_IIC_Send_Byte(reg);					// Specify target register
    MPU_IIC_Wait_Ack();
    MPU_IIC_Start();
		MPU_IIC_Send_Byte((addr<<1)|1);		// Send address + read flag
    MPU_IIC_Wait_Ack();
		while(len)
		{
			if(len==1)
				*buf=MPU_IIC_Read_Byte(0);   // Last byte: send NACK
			else
				*buf=MPU_IIC_Read_Byte(1);	 // Not last byte: send ACK
			len--;
			buf++;
		}
    MPU_IIC_Stop();
		return 0;
}

/**
 * @brief  Single byte register write
 * @param  reg: target register address
 * @param  data: byte data to write
 * @retval 0: write ok; 1: fail
 */
uint8_t MPU_Write_Byte(uint8_t reg,uint8_t data)
{
  MPU_IIC_Start();
	MPU_IIC_Send_Byte((MPU_ADDR<<1)|0);
	if(MPU_IIC_Wait_Ack())
	{
		MPU_IIC_Stop();
		return 1;
	}
  MPU_IIC_Send_Byte(reg);
  MPU_IIC_Wait_Ack();
	MPU_IIC_Send_Byte(data);
	if(MPU_IIC_Wait_Ack())
	{
		MPU_IIC_Stop();
		return 1;
	}
  MPU_IIC_Stop();
	return 0;
}

/**
 * @brief  Read single byte from register
 * @param  reg: target register address
 * @retval received byte value
 */
uint8_t MPU_Read_Byte(uint8_t reg)
{
	uint8_t res;

  MPU_IIC_Start();
	MPU_IIC_Send_Byte((MPU_ADDR<<1)|0);
	MPU_IIC_Wait_Ack();
  MPU_IIC_Send_Byte(reg);
  MPU_IIC_Wait_Ack();
  MPU_IIC_Start();
	MPU_IIC_Send_Byte((MPU_ADDR<<1)|1);
  MPU_IIC_Wait_Ack();
	res=MPU_IIC_Read_Byte(0); // Read byte, send NACK
  MPU_IIC_Stop();
	return res;
}
