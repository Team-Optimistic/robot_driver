#include "robot_driver/MPU6000.h"
#include <wiringPiSPI.h>
#include <iostream>
#include <unistd.h>


mpu6000::mpu6000(int csChannel, long speed)
{
  channel = csChannel;
  wiringPiSPISetup(channel, speed);
}

/*-----------------------------------------------------------------------------------------------
                                    INITIALIZATION
usage: call this function at startup, giving the sample rate divider (raging from 0 to 255) and
low pass filter value; suitable values are:
BITS_DLPF_CFG_256HZ_NOLPF2
BITS_DLPF_CFG_188HZ
BITS_DLPF_CFG_98HZ
BITS_DLPF_CFG_42HZ
BITS_DLPF_CFG_20HZ
BITS_DLPF_CFG_10HZ
BITS_DLPF_CFG_5HZ
BITS_DLPF_CFG_2100HZ_NOLPF
returns 1 if an error occurred
-----------------------------------------------------------------------------------------------*/

unsigned char mpu6000::write(unsigned char dataIn)
{
  unsigned char buff[1] = {dataIn};
  wiringPiSPIDataRW (channel, buff, 1);
  return buff[0];
}

unsigned char mpu6000::writeReg(unsigned char reg, unsigned char value)
{
	unsigned char buf[2] = {reg, value};
	wiringPiSPIDataRW(channel, buf, 2);
	return buf[0];
}

void mpu6000::wakeup()
{
	unsigned char response;

	write(MPUREG_PWR_MGMT_1 | READ_FLAG);
	response = write(0x00);

	std::cout << "wakeup:"<< std::hex << (int)response << std::endl;

	for (int i = 0; i < 1; i++)
	{
		usleep(500000);
	}

	writeReg(MPUREG_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ);

	for (int i = 0; i < 1; i++)
	{
		usleep(500000);
	}

	write(MPUREG_PWR_MGMT_1 | READ_FLAG);
	response = write(0x00);

	std::cout << "new wakeup:" << std::hex << (int)response << std::endl;
}

bool mpu6000::init(int sample_rate_div,int low_pass_filter)
{
  unsigned int response;

  //FIRST OF ALL DISABLE I2C
  writeReg(MPUREG_USER_CTRL, BIT_I2C_IF_DIS);

  //RESET CHIP
  writeReg(MPUREG_PWR_MGMT_1, BIT_H_RESET);
  usleep(150000);

  //DISABLE I2C
  writeReg(MPUREG_USER_CTRL, BIT_I2C_IF_DIS);

  //WHO AM I?
   wakeup();
  response = write(MPUREG_WHOAMI | READ_FLAG);
  response = write(0x00);

  std::cout<<"whoami:"<<response<<std::endl;

  if (response < 100) { return 0; } //COULDN'T RECEIVE WHOAMI

  //SET SAMPLE RATE
  writeReg(MPUREG_SMPLRT_DIV, sample_rate_div);

  // FS & DLPF
  writeReg(MPUREG_CONFIG, low_pass_filter);

  //DISABLE INTERRUPTS
  writeReg(MPUREG_INT_ENABLE, 0x00);

  return 0;
}

/*-----------------------------------------------------------------------------------------------
                                ACCELEROMETER SCALE
usage: call this function at startup, after initialization, to set the right range for the
accelerometers. Suitable ranges are:
BITS_FS_2G
BITS_FS_4G
BITS_FS_8G
BITS_FS_16G
returns the range set (2,4,8 or 16)
-----------------------------------------------------------------------------------------------*/
unsigned int mpu6000::set_acc_scale(int scale)
{
  unsigned int temp_scale;

  writeReg(MPUREG_ACCEL_CONFIG, scale);

  switch (scale)
  {
    case BITS_FS_2G:
      acc_divider = 16384;
      break;
    case BITS_FS_4G:
      acc_divider = 8192;
      break;
    case BITS_FS_8G:
      acc_divider = 4096;
      break;
    case BITS_FS_16G:
      acc_divider = 2048;
      break;
    default:
      acc_divider = 2048;
      break;
    }

  usleep(10000);

  temp_scale = write(MPUREG_ACCEL_CONFIG | READ_FLAG);
  temp_scale = write(0x00);

  switch (temp_scale)
  {
    case BITS_FS_2G:
      temp_scale = 2;
      break;
    case BITS_FS_4G:
      temp_scale = 4;
      break;
    case BITS_FS_8G:
      temp_scale = 8;
      break;
    case BITS_FS_16G:
      temp_scale = 16;
      break;
  	default:
      temp_scale = 16;
      break;
    }

  return temp_scale;
}


/*-----------------------------------------------------------------------------------------------
                                GYROSCOPE SCALE
usage: call this function at startup, after initialization, to set the right range for the
gyroscopes. Suitable ranges are:
BITS_FS_250DPS
BITS_FS_500DPS
BITS_FS_1000DPS
BITS_FS_2000DPS
returns the range set (250,500,1000 or 2000)
-----------------------------------------------------------------------------------------------*/
unsigned int mpu6000::set_gyro_scale(int scale)
{
  unsigned int temp_scale;

  writeReg(MPUREG_GYRO_CONFIG, scale);

  switch (scale)
  {
    case BITS_FS_250DPS:
        gyro_divider = 131;
        break;
    case BITS_FS_500DPS:
        gyro_divider = 65.5;
        break;
    case BITS_FS_1000DPS:
        gyro_divider = 32.8;
        break;
    case BITS_FS_2000DPS:
        gyro_divider = 16.4;
        break;
  	default:
    	gyro_divider = 16.4;
    	break;
  }

	usleep(10000);

  temp_scale = write(MPUREG_GYRO_CONFIG | READ_FLAG);
  temp_scale = write(0x00);

  switch (temp_scale)
  {
    case BITS_FS_250DPS:
      temp_scale = 250;
      break;
    case BITS_FS_500DPS:
      temp_scale = 500;
      break;
    case BITS_FS_1000DPS:
      temp_scale = 1000;
      break;
    case BITS_FS_2000DPS:
      temp_scale = 2000;
      break;
  	default:
    	temp_scale = 2000;
    	break;
    }

  return temp_scale;
}


/*-----------------------------------------------------------------------------------------------
                                WHO AM I?
usage: call this function to know if SPI is working correctly. It checks the I2C address of the
mpu6000 which should be 104 when in SPI mode.
returns the I2C address (104)
-----------------------------------------------------------------------------------------------*/
unsigned int mpu6000::whoami()
{
  unsigned int response;

  response = write(MPUREG_WHOAMI | READ_FLAG);
  response = write(0x00);

  return response;
}


/*-----------------------------------------------------------------------------------------------
                                READ ACCELEROMETER
usage: call this function to read accelerometer data. Axis represents selected axis:
0 -> X axis
1 -> Y axis
2 -> Z axis
returns the value in Gs
-----------------------------------------------------------------------------------------------*/
float mpu6000::read_acc(int axis)
{
  unsigned char responseH, responseL;
  int16_t bit_data;
  float data;

  switch (axis)
  {
    case 0:
      responseH = write(MPUREG_ACCEL_XOUT_H | READ_FLAG);
      responseH = write(0x00);
      responseL = write(MPUREG_ACCEL_XOUT_L | READ_FLAG);
      responseL = write(0x00);
      break;
    case 1:
      responseH = write(MPUREG_ACCEL_YOUT_H | READ_FLAG);
      responseH = write(0x00);
      responseL = write(MPUREG_ACCEL_YOUT_L | READ_FLAG);
      responseL = write(0x00);
      break;
    case 2:
      responseH = write(MPUREG_ACCEL_ZOUT_H | READ_FLAG);
      responseH = write(0x00);
      responseL = write(MPUREG_ACCEL_ZOUT_L | READ_FLAG);
      responseL = write(0x00);
      break;
    default:
      responseH = write(MPUREG_ACCEL_XOUT_H | READ_FLAG);
      responseH = write(0x00);
      responseL = write(MPUREG_ACCEL_XOUT_L | READ_FLAG);
      responseL = write(0x00);
      break;
  }

  bit_data = responseH;
  bit_data = (bit_data << 8) | responseL;
  data=(float)bit_data;
  data=(float)data / (float)acc_divider;

  return data;
}

/*-----------------------------------------------------------------------------------------------
                                READ GYROSCOPE
usage: call this function to read gyroscope data. Axis represents selected axis:
0 -> X axis
1 -> Y axis
2 -> Z axis
returns the value in Degrees per second
-----------------------------------------------------------------------------------------------*/
float mpu6000::read_rot(int axis)
{
    unsigned char responseH, responseL;
    int16_t bit_data;
    float data;

    switch (axis)
    {
      case 0:
  	    responseH = write(MPUREG_GYRO_XOUT_H | READ_FLAG);
  			responseH = write(0x00);
  	    responseL = write(MPUREG_GYRO_XOUT_L | READ_FLAG);
  			responseL = write(0x00);
  	    break;
      case 1:
  			responseH = write(MPUREG_GYRO_YOUT_H | READ_FLAG);
  			responseH = write(0x00);
  	    responseL = write(MPUREG_GYRO_YOUT_L | READ_FLAG);
  			responseL = write(0x00);
  	    break;
  		case 2:
  			responseH = write(MPUREG_GYRO_ZOUT_H | READ_FLAG);
  			responseH = write(0x00);
  	    responseL = write(MPUREG_GYRO_ZOUT_L | READ_FLAG);
  			responseL = write(0x00);
  	    break;
  		default:
  			responseH = write(MPUREG_GYRO_XOUT_H | READ_FLAG);
  			responseH = write(0x00);
  	    responseL = write(MPUREG_GYRO_XOUT_L | READ_FLAG);
  			responseL = write(0x00);
  			break;
    }

    bit_data = responseH;
    bit_data = (bit_data << 8) | responseL;
    data=(float)bit_data;
    data=(float)data / (float)gyro_divider;

    return data;
}

/*-----------------------------------------------------------------------------------------------
                                READ TEMPERATURE
usage: call this function to read temperature data.
returns the value in °C
-----------------------------------------------------------------------------------------------*/
float mpu6000::read_temp()
{
  unsigned char responseH, responseL;
  int16_t bit_data;
  float data;

  responseH = write(MPUREG_TEMP_OUT_H | READ_FLAG);
  responseH = write(0x00);
  responseL = write(MPUREG_TEMP_OUT_L | READ_FLAG);
  responseL = write(0x00);
  bit_data = responseH;
  bit_data = (bit_data << 8) | responseL;

  data = (float)bit_data;
  data = (data / 340.0) + 36.53;

  return data;
}

/*-----------------------------------------------------------------------------------------------
                                READ ACCELEROMETER CALIBRATION
usage: call this function to read accelerometer data. Axis represents selected axis:
0 -> X axis
1 -> Y axis
2 -> Z axis
returns Factory Trim value
-----------------------------------------------------------------------------------------------*/
int mpu6000::calib_acc(int axis)
{
    unsigned char responseH, responseL, calib_data;
    int temp_scale;

    //READ CURRENT ACC SCALE
    responseH = write(MPUREG_ACCEL_CONFIG | READ_FLAG);
    temp_scale = write(0x00);

    set_acc_scale(BITS_FS_8G);

    //ENABLE SELF TEST
    responseH = write(MPUREG_ACCEL_CONFIG);
    temp_scale = write(0x80 >> axis);
    responseH = write(MPUREG_SELF_TEST_X | READ_FLAG);

    switch (axis)
    {
      case 0:
        responseH = write(0x00);
        responseL = write(0x00);
        responseL = write(0x00);
        responseL = write(0x00);
        calib_data = ((responseH&11100000) >> 3) | ((responseL&00110000) >> 4);
      break;
      case 1:
        responseH = write(0x00);
        responseH = write(0x00);
        responseL = write(0x00);
        responseL = write(0x00);
        calib_data = ((responseH&11100000) >> 3) | ((responseL&00001100) >> 2);
      break;
      case 2:
        responseH = write(0x00);
        responseH = write(0x00);
        responseH = write(0x00);
        responseL = write(0x00);
        calib_data = ((responseH&11100000) >> 3) | ((responseL&00000011));
      break;
      default:
        responseH = write(0x00);
        responseL = write(0x00);
        responseL = write(0x00);
        responseL = write(0x00);
        calib_data = ((responseH&11100000) >> 3) | ((responseL&00110000) >> 4);
        break;
    }

    set_acc_scale(temp_scale);
    return calib_data;
}
