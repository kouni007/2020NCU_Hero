#include "MPU6500.h"


uint8_t MPU_id = 0;
int16_t gy_data_filter[5];
int16_t gz_data_filter[5];

IMUDataTypedef imu_data = {0,0,0,0,0,0,0,0,0,0};
IMUDataTypedef imu_data_offest = {0,0,0,0,0,0,0,0,0,0};


/*************************°åÔØimuÄ£¿é*****************************/
/***************************************************************************************
**
	*	@brief	 MPU6500_Write_Reg(uint8_t const reg, uint8_t const data)
	*	@param
	*	@supplement	Write a register to MPU6500
	*	@retval	
****************************************************************************************/
uint8_t MPU6500_Write_Reg(uint8_t const reg, uint8_t const data)
{
  static uint8_t MPU_Rx, MPU_Tx;
  
  MPU6500_NSS_Low();
  
  MPU_Tx = reg&0x7f;
  HAL_SPI_TransmitReceive(&hspi5, &MPU_Tx, &MPU_Rx, 1, 55);
  MPU_Tx = data;
  HAL_SPI_TransmitReceive(&hspi5, &MPU_Tx, &MPU_Rx, 1, 55);
  
  MPU6500_NSS_High();
  return 0;
}

/***************************************************************************************
**
	*	@brief	 MPU6500_Read_Reg(uint8_t const reg)
	*	@param
	*	@supplement	Read a register from MPU6500
	*	@retval	
****************************************************************************************/
uint8_t MPU6500_Read_Reg(uint8_t const reg)
{
  static uint8_t MPU_Rx, MPU_Tx;
  
  MPU6500_NSS_Low();
  
  MPU_Tx = reg|0x80;
  HAL_SPI_TransmitReceive(&hspi5, &MPU_Tx, &MPU_Rx, 1, 55);
  HAL_SPI_TransmitReceive(&hspi5, &MPU_Tx, &MPU_Rx, 1, 55);
  
  MPU6500_NSS_High();
  return MPU_Rx;
}

/***************************************************************************************
**
	*	@brief	 MPU6500_Read_Regs(uint8_t const regAddr, uint8_t *pData, uint8_t len)
	*	@param
	*	@supplement	Read registers from MPU6500,address begin with regAddr
	*	@retval	
****************************************************************************************/
uint8_t MPU6500_Read_Regs(uint8_t const regAddr, uint8_t *pData, uint8_t len)
{
  static uint8_t MPU_Rx, MPU_Tx, MPU_Tx_buff[14] = {0xff};
  MPU6500_NSS_Low();
  
  MPU_Tx = regAddr|0x80;
  MPU_Tx_buff[0] = MPU_Tx;
  HAL_SPI_TransmitReceive(&hspi5, &MPU_Tx, &MPU_Rx, 1, 55);
  HAL_SPI_TransmitReceive(&hspi5, MPU_Tx_buff, pData, len, 55);
  
  MPU6500_NSS_High();
  return 0;
}

/***************************************************************************************
**
	*	@brief	 IST_Reg_Write_By_MPU(uint8_t addr, uint8_t data)
	*	@param
	*	@supplement	Write IST8310 register through MPU6500
	*	@retval	
****************************************************************************************/
static void IST_Reg_Write_By_MPU(uint8_t addr, uint8_t data)
{
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x00);
  HAL_Delay(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_REG, addr);
  HAL_Delay(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_DO, data);
  HAL_Delay(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x080 | 0x01);
  HAL_Delay(10);
}

/***************************************************************************************
**
	*	@brief	 IST_Reg_Read_By_MPU(uint8_t addr)
	*	@param
	*	@supplement	Write IST8310 register through MPU6500
	*	@retval	
****************************************************************************************/
uint8_t IST_Reg_Read_By_MPU(uint8_t addr)
{
  uint8_t data;
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_REG, addr);
  HAL_Delay(10);
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x80);
  HAL_Delay(10);
  data = MPU6500_Read_Reg(MPU6500_I2C_SLV4_DI);
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x00);
  HAL_Delay(10);
  return data;
}

/***************************************************************************************
**
	*	@brief	 MPU_Auto_Read_IST_config(uint8_t device_address, uint8_t reg_base_addr, uint8_t data_num)
	*	@param
	*	@supplement	Initialize the MPU6500 I2C Slave0 for I2C reading
	*	@retval	
****************************************************************************************/
static void MPU_Auto_Read_IST_config(uint8_t device_address, uint8_t reg_base_addr, uint8_t data_num)
{
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_ADDR, device_address);
  HAL_Delay(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_REG, IST8310_R_CONFA);
  HAL_Delay(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_DO, IST8310_ODR_MODE);
  HAL_Delay(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV0_ADDR, 0x80 | device_address);
  HAL_Delay(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV0_REG, reg_base_addr);
  HAL_Delay(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x03);
  HAL_Delay(2);
  
//  MPU6500_Write_Reg(MPU6500_I2C_MST_DELAY_CTRL, 0x01 | 0x02);
	  MPU6500_Write_Reg(MPU6500_I2C_MST_DELAY_CTRL, 0x00);

  HAL_Delay(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
  HAL_Delay(6);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV0_CTRL, 0x80 | data_num);
  HAL_Delay(7);
}

/***************************************************************************************
**
	*	@brief	 IST8310_Init(void)
	*	@param
	*	@supplement	Initialize the IST8310
	*	@retval	
****************************************************************************************/
uint8_t IST8310_Init(void)
{
  MPU6500_Write_Reg(MPU6500_USER_CTRL, 0x30);
  HAL_Delay(10);
  MPU6500_Write_Reg(MPU6500_I2C_MST_CTRL, 0x0d);
  HAL_Delay(10);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_ADDR, IST8310_ADDRESS);
  HAL_Delay(10);
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_ADDR, 0x80 | IST8310_ADDRESS);
  HAL_Delay(10);
  
  IST_Reg_Write_By_MPU(IST8310_R_CONFB, 0x01);
  if(IST8310_DEVICE_ID_A != IST_Reg_Read_By_MPU(IST8310_WHO_AM_I))
    return 1; //error
  HAL_Delay(10);
  
  IST_Reg_Write_By_MPU(IST8310_R_CONFA, 0x00);
  if(IST_Reg_Read_By_MPU(IST8310_R_CONFA) != 0x00)
    return 2;
  HAL_Delay(10);
  
  IST_Reg_Write_By_MPU(IST8310_R_CONFB, 0x00);
  if(IST_Reg_Read_By_MPU(IST8310_R_CONFB) != 0x00)
    return 3;
  HAL_Delay(10);
  
  IST_Reg_Write_By_MPU(IST8310_AVGCNTL, 0x24);
  if(IST_Reg_Read_By_MPU(IST8310_AVGCNTL) != 0x24)
    return 4;
  HAL_Delay(10);
  
  IST_Reg_Write_By_MPU(IST8310_PDCNTL, 0xc0);
  if(IST_Reg_Read_By_MPU(IST8310_PDCNTL) != 0xc0)
    return 5;
  HAL_Delay(10);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x00);
  HAL_Delay(10);
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x00);
  HAL_Delay(10);
  
  MPU_Auto_Read_IST_config(IST8310_ADDRESS, IST8310_R_XL, 0x06);
  HAL_Delay(100);
  return 0;
}



//Set the accelerated velocity resolution
uint8_t MPU6500_Set_Accel_Fsr(uint8_t fsr)
{
  return MPU6500_Write_Reg(MPU6500_ACCEL_CONFIG, fsr<<3);
}

//Set the angular velocity resolution
uint8_t MPU6500_Set_Gyro_Fsr(uint8_t fsr)
{
  return MPU6500_Write_Reg(MPU6500_GYRO_CONFIG, fsr<<3);
}

/***************************************************************************************
**
	*	@brief	IMU_Get_Data()
	*	@param
	*	@supplement	ÁùÖáËã·¨
	*	@retval	
****************************************************************************************/
void IMU_Get_Data()
{
	uint8_t i;
  int16_t data_sum = 0;
  uint8_t mpu_buff[14];
  MPU6500_Read_Regs(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);
  
  imu_data.ax = mpu_buff[0] << 8 | mpu_buff[1];
  imu_data.ay = mpu_buff[2] << 8 | mpu_buff[3];
  imu_data.az = mpu_buff[4] << 8 | mpu_buff[5];
  
  imu_data.temp = mpu_buff[6] << 8 | mpu_buff[7];
  
  imu_data.gx = mpu_buff[8] << 8 | mpu_buff[9];//- imu_data_offest.gx;
  imu_data.gy = mpu_buff[10] << 8 | mpu_buff[11];//- imu_data_offest.gy;
  imu_data.gz = mpu_buff[12] << 8 | mpu_buff[13];//- imu_data_offest.gz;

	gy_data_filter[4] = gy_data_filter[3];
	gy_data_filter[3] = gy_data_filter[2];
	gy_data_filter[2] = gy_data_filter[1];
	gy_data_filter[1] = gy_data_filter[0];
  gy_data_filter[0] = imu_data.gy;
	for(i = 0;i < 5;i++)
	{
		data_sum += gy_data_filter[i];
	}
	imu_data.gy = data_sum / 5;

	data_sum = 0;
	
	gz_data_filter[4] = gz_data_filter[3];
	gz_data_filter[3] = gz_data_filter[2];
	gz_data_filter[2] = gz_data_filter[1];
	gz_data_filter[1] = gz_data_filter[0];
  gz_data_filter[0] = imu_data.gz;
	for(i = 0;i < 5;i++)
	{
		data_sum += gz_data_filter[i];
	}
	imu_data.gz = data_sum / 5;
	
//  imu_data.gz = LPF_1st(imu_data.last_gz,imu_data.gz,0.7);//Ò»½×µÍÍ¨ÂË²¨
	
	
	imu_data.last_gz = imu_data.gz;
}





uint8_t MPU6500_Init(void)
{
	static float raw_gz[6],raw_gy[6];
	uint8_t i;
	static float sum_gz,sum_gy;
  uint8_t index = 0;
  uint8_t MPU6500_Init_Data[10][2] = 
  {
    {MPU6500_PWR_MGMT_1,    0x80},      // Reset Device
    {MPU6500_PWR_MGMT_1,    0x03},      // Clock Source - Gyro-Z
    {MPU6500_PWR_MGMT_2,    0x00},      // Enable Acc & Gyro
    {MPU6500_CONFIG,        0x02},      // LPF 98Hz
    {MPU6500_GYRO_CONFIG,   0x18},      // +-2000dps
    {MPU6500_ACCEL_CONFIG,  0x10},      // +-8G
    {MPU6500_ACCEL_CONFIG_2,0x02},      // enable LowPassFilter  Set Acc LPF
    {MPU6500_USER_CTRL,     0x20},      // Enable AUX
  };
  
  HAL_Delay(100);
  MPU_id = MPU6500_Read_Reg(MPU6500_WHO_AM_I);  //read id of device,check if MPU6500 or not
  
  for(index = 0; index < 10; index++)
  {
    MPU6500_Write_Reg(MPU6500_Init_Data[index][0], MPU6500_Init_Data[index][1]);
    HAL_Delay(1);
  }
	
//ÁãÆ«Á¿
	
	
	for(i=0;i<6;i++)
	{
		IMU_Get_Data();
		raw_gz[i]=imu_data.gz;
		raw_gy[i]=imu_data.gy;
		
		sum_gz += raw_gz[i];
		sum_gy += raw_gy[i];
		
		if(i==5)
		{
			imu_data_offest.gz = sum_gz / 6.0f;
			imu_data_offest.gy = sum_gy / 6.0f;
			
			sum_gz = 0;
			sum_gy = 0;
			
		}
	}

  return 0;
}
