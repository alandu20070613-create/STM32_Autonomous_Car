#include "mpuiic.h"
#include "delay.h"

static GPIO_InitTypeDef GPIO_InitStruct;

/**
 * @brief  Configure SDA pin as floating input
 * @retval None
 */
void HAL_GPIO_Init_SDA_INPUT(void)
{
    GPIO_InitStruct.Pin = SDA_6050_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(SDA_6050_GPIO_Port, &GPIO_InitStruct);
}

/**
 * @brief  Configure SDA pin as push-pull output
 * @retval None
 */
void HAL_GPIO_Init_SDA_OUTPUT(void)
{
    GPIO_InitStruct.Pin = SDA_6050_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SDA_6050_GPIO_Port, &GPIO_InitStruct);
}

/**
 * @brief  Microsecond precise delay for IIC timing
 * @param  usdelay: delay time in microseconds
 * @retval None
 */
void mpuiic_Delayus(uint32_t usdelay)
{
	__IO uint32_t Delay = usdelay * (SystemCoreClock / 8U / 1000U / 1000);
	do{
		__NOP();
	}
	while (Delay--);
}

/**
 * @brief  Fixed 2us delay for IIC timing requirements
 * @retval None
 */
void MPU_IIC_Delay(void)
{
	mpuiic_Delayus(2);
}

/**
 * @brief  Initialize PB10(SCL) PB11(SDA) for software IIC
 * @retval None
 */
void MPU_IIC_Init(void)
{
	// Enable GPIOB peripheral clock
	__HAL_RCC_GPIOB_CLK_ENABLE();

	// Configure SCL as push-pull output
	GPIO_InitStruct.Pin = SCL_6050_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(SCL_6050_GPIO_Port, &GPIO_InitStruct);

	// Initialize SDA to output mode
	HAL_GPIO_Init_SDA_OUTPUT();

	// IIC bus idle level: SCL=High, SDA=High
	MPU_IIC_SCL_HIGH;
	MPU_IIC_SDA_HIGH;
}

/**
 * @brief  Generate IIC start signal
 * @note   SCL high, SDA falling edge = start
 * @retval None
 */
void MPU_IIC_Start(void)
{
	MPU_SDA_OUT();
	MPU_IIC_SDA_HIGH;
	MPU_IIC_SCL_HIGH;
	MPU_IIC_Delay();
 	MPU_IIC_SDA_LOW;
	MPU_IIC_Delay();
	MPU_IIC_SCL_LOW; // Hold SCL low to occupy bus
}

/**
 * @brief  Generate IIC stop signal
 * @note   SCL high, SDA rising edge = stop
 * @retval None
 */
void MPU_IIC_Stop(void)
{
	MPU_SDA_OUT();
	MPU_IIC_SCL_LOW;
	MPU_IIC_SDA_LOW;
 	MPU_IIC_Delay();
	MPU_IIC_SCL_HIGH;
	MPU_IIC_SDA_HIGH;
	MPU_IIC_Delay();
}

/**
 * @brief  Master waits for slave ACK response
 * @retval 0: ACK received successfully; 1: timeout no ACK
 */
uint8_t MPU_IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
	MPU_SDA_IN();  // Switch SDA to input for reading slave level
	MPU_IIC_SDA_HIGH;
	MPU_IIC_Delay();
	MPU_IIC_SCL_HIGH;
	MPU_IIC_Delay();
	while(MPU_READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			MPU_IIC_Stop();
			return 1;
		}
	}
	MPU_IIC_SCL_LOW; // Pull SCL low to continue communication
	return 0;
}

/**
 * @brief  Master sends ACK signal to slave
 * @retval None
 */
void MPU_IIC_Ack(void)
{
	MPU_IIC_SCL_LOW;
	MPU_SDA_OUT();
	MPU_IIC_SDA_LOW;
	MPU_IIC_Delay();
	MPU_IIC_SCL_HIGH;
	MPU_IIC_Delay();
	MPU_IIC_SCL_LOW;
}

/**
 * @brief  Master sends NACK signal to slave
 * @retval None
 */
void MPU_IIC_NAck(void)
{
	MPU_IIC_SCL_LOW;
	MPU_SDA_OUT();
	MPU_IIC_SDA_HIGH;
	MPU_IIC_Delay();
	MPU_IIC_SCL_HIGH;
	MPU_IIC_Delay();
	MPU_IIC_SCL_LOW;
}

/**
 * @brief  Send one byte over IIC, MSB first
 * @param  txd: byte data to transmit
 * @retval None
 */
void MPU_IIC_Send_Byte(uint8_t txd)
{
    uint8_t t;
	MPU_SDA_OUT();
    MPU_IIC_SCL_LOW; // Pull clock low before bit transmission
    for(t=0;t<8;t++)
    {
        if((txd&0x80)>>7)
			MPU_IIC_SDA_HIGH;
		else
			MPU_IIC_SDA_LOW;
        txd<<=1;
		MPU_IIC_SCL_HIGH;
		MPU_IIC_Delay();
		MPU_IIC_SCL_LOW;
		MPU_IIC_Delay();
    }
}

/**
 * @brief  Read one byte from IIC bus
 * @param  ack: 1=send ACK after read, 0=send NACK after read
 * @retval Received byte data
 */
uint8_t MPU_IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
    MPU_SDA_IN(); // Set SDA as input to read data
    for(i=0;i<8;i++)
	{
        MPU_IIC_SCL_LOW;
        MPU_IIC_Delay();
		MPU_IIC_SCL_HIGH;
        receive<<=1;
        if(MPU_READ_SDA)receive++;
		MPU_IIC_Delay();
    }
    if (!ack)
        MPU_IIC_NAck();
    else
        MPU_IIC_Ack();
    return receive;
}

/**
 * @brief  Write single byte to specified register of IIC device
 * @param  daddr: device IIC address
 * @param  addr: target register address
 * @param  data: byte data to write
 * @retval None
 */
void MPU_IIC_Write_One_Byte(uint8_t daddr,uint8_t addr,uint8_t data)
{
	MPU_IIC_Start();
	MPU_IIC_Send_Byte((daddr << 1) | 0);
	if(MPU_IIC_Wait_Ack())
	{
		MPU_IIC_Stop();
		return;
	}
	MPU_IIC_Send_Byte(addr);
	if(MPU_IIC_Wait_Ack())
	{
		MPU_IIC_Stop();
		return;
	}
	MPU_IIC_Send_Byte(data);
	MPU_IIC_Wait_Ack();
	MPU_IIC_Stop();
}

/**
 * @brief  Read single byte from specified register of IIC device
 * @param  daddr: device IIC address
 * @param  addr: target register address
 * @retval Readback byte
 */
uint8_t MPU_IIC_Read_One_Byte(uint8_t daddr,uint8_t addr)
{
	uint8_t val = 0;
	MPU_IIC_Start();
	MPU_IIC_Send_Byte((daddr << 1) | 0);
	if(MPU_IIC_Wait_Ack())
	{
		MPU_IIC_Stop();
		return 0;
	}
	MPU_IIC_Send_Byte(addr);
	MPU_IIC_Wait_Ack();

	MPU_IIC_Start();
	MPU_IIC_Send_Byte((daddr << 1) | 1);
	MPU_IIC_Wait_Ack();

	val = MPU_IIC_Read_Byte(0);
	MPU_IIC_Stop();
	return val;
}
