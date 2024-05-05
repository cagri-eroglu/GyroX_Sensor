
#ifndef SRC_IMU_H_
#define SRC_IMU_H_

#include "stm32f4xx_hal.h"



// LIS3MDL Digital output magnetic sensor:
// http://www.st.com/web/catalog/sense_power/FM89/SC1449/PF255198
//
//Magnetometer Registers
#define LIS3MDL_ADDRESS      0x1C
#define LIS3MDL_WHO_AM_I     0x0F  // should return 0x3D
#define LIS3MDL_CTRL_REG1    0x20
#define LIS3MDL_CTRL_REG2    0x21
#define LIS3MDL_CTRL_REG3    0x22
#define LIS3MDL_CTRL_REG4    0x23
#define LIS3MDL_CTRL_REG5    0x24
#define LIS3MDL_STATUS_REG   0x27
#define LIS3MDL_OUT_X_L	     0x28  // data
#define LIS3MDL_OUT_X_H	     0x29
#define LIS3MDL_OUT_Y_L	     0x2A
#define LIS3MDL_OUT_Y_H	     0x2B
#define LIS3MDL_OUT_Z_L	     0x2C
#define LIS3MDL_OUT_Z_H	     0x2D
#define LIS3MDL_TEMP_OUT_L   0x2E
#define LIS3MDL_TEMP_OUT_H   0x2F  // data
#define LIS3MDL_INT_CFG	     0x30
#define LIS3MDL_INT_SRC	     0x31
#define LIS3MDL_INT_THS_L    0x32
#define LIS3MDL_INT_THS_H    0x33

// MAX21100 Register Map
// http://www.maximintegrated.com/en/products/analog/sensors-and-sensor-interface/MAX21100.html
//
// MAX21100 Common Bank
#define MAX21100_WHO_AM_I      0x20    // should be 0xB2
#define MAX21100_REVISION_ID   0x21
#define MAX21100_BANK_SELECT   0x22
#define MAX21100_SYSTEM_STATUS 0x23
#define MAX21100_GYRO_X_H      0x24
#define MAX21100_GYRO_X_L      0x25
#define MAX21100_GYRO_Y_H      0x26
#define MAX21100_GYRO_Y_L      0x27
#define MAX21100_GYRO_Z_H      0x28
#define MAX21100_GYRO_Z_L      0x29
#define MAX21100_ACC_X_H       0x2A
#define MAX21100_ACC_X_L       0x2B
#define MAX21100_ACC_Y_H       0x2C
#define MAX21100_ACC_Y_L       0x2D
#define MAX21100_ACC_Z_H       0x2E
#define MAX21100_ACC_Z_L       0x2F
#define MAX21100_MAG_X_H       0x30
#define MAX21100_MAG_X_L       0x31
#define MAX21100_MAG_Y_H       0x32
#define MAX21100_MAG_Y_L       0x33
#define MAX21100_MAG_Z_H       0x34
#define MAX21100_MAG_Z_L       0x35
#define MAX21100_TEMP_H        0x36
#define MAX21100_TEMP_L        0x37
#define MAX21100_FIFO_COUNT    0x3C
#define MAX21100_FIFO_STATUS   0x3D
#define MAX21100_FIFO_DATA     0x3E
#define MAX21100_RST_REG       0x3F

// MAX21100 Bank 0
#define MAX21100_POWER_CFG     0x00
#define MAX21100_GYRO_CFG1     0x01
#define MAX21100_GYRO_CFG2     0x02
#define MAX21100_GYRO_CFG3     0x03
#define MAX21100_PWR_ACC_CFG   0x04
#define MAX21100_ACC_CFG1      0x05
#define MAX21100_ACC_CFG2      0x06
#define MAX21100_MAG_SLV_CFG   0x07
#define MAX21100_MAG_SLV_ADD   0x08
#define MAX21100_MAG_SLV_REG   0x09
#define MAX21100_MAG_MAP_REG   0x0A
#define MAX21100_I2C_MST_ADD   0x0B
#define MAX21100_I2C_MST_DATA  0x0C
#define MAX21100_MAG_OFS_X_MSB 0x0D
#define MAX21100_MAG_OFS_X_LSB 0x0E
#define MAX21100_MAG_OFS_Y_MSB 0x0F
#define MAX21100_MAG_OFS_Y_LSB 0x10
#define MAX21100_MAG_OFS_Z_MSB 0x11
#define MAX21100_MAG_OFS_Z_LSB 0x12
#define MAX21100_DR_CFG        0x13
#define MAX21100_IO_CFG        0x14
#define MAX21100_I2C_PAD       0x15
#define MAX21100_I2C_CFG       0x16
#define MAX21100_FIFO_TH       0x17
#define MAX21100_FIFO_CFG      0x18
#define MAX21100_DSYNC_CFG     0x1A
#define MAX21100_DSYNC_CNT     0x1B
#define MAX21100_ITF_OTP       0x1C

// MAX21100 Bank 1
#define MAX21100_INT_REF_X     0x00
#define MAX21100_INT_REF_Y     0x01
#define MAX21100_INT_REF_Z     0x02
#define MAX21100_INT_DEB_X     0x03
#define MAX21100_INT_DEB_Y     0x04
#define MAX21100_INT_DEB_Z     0x05
#define MAX21100_INT_MSK_X     0x06
#define MAX21100_INT_MSK_Y     0x07
#define MAX21100_INT_MSK_Z     0x08
#define MAX21100_INT_MSK_AQ    0x09
#define MAX21100_INT_CFG1      0x0A
#define MAX21100_INT_CFG2      0x0B
#define MAX21100_INT_TM0       0x0C
#define MAX21100_INT_STS_UL    0x0D
#define MAX21100_INT_STS       0x0E
#define MAX21100_INT_MSK       0x0F
#define MAX21100_INT_SRC_SEL   0x17
#define MAX21100_SERIAL_5      0x1A
#define MAX21100_SERIAL_4      0x1B
#define MAX21100_SERIAL_3      0x1C
#define MAX21100_SERIAL_2      0x1D
#define MAX21100_SERIAL_1      0x1E
#define MAX21100_SERIAL_0      0x1F

// MAX21100 Bank 2 (Bank select 0010)

#define MAX21100_QUAT0_H         0x00
#define MAX21100_QUAT0_L         0x01
#define MAX21100_QUAT1_H         0x02
#define MAX21100_QUAT1_L         0x03
#define MAX21100_QUAT2_H         0x04
#define MAX21100_QUAT2_L         0x05
#define MAX21100_QUAT3_H         0x06
#define MAX21100_QUAT3_L         0x07
#define MAX21100_BIAS_GYRO_X_H   0x13
#define MAX21100_BIAS_GYRO_X_L   0x14
#define MAX21100_BIAS_GYRO_Y_H   0x15
#define MAX21100_BIAS_GYRO_Y_L   0x16
#define MAX21100_BIAS_GYRO_Z_H   0x17
#define MAX21100_BIAS_GYRO_Z_L   0x18
#define MAX21100_BIAS_COMP_ACC_X 0x19
#define MAX21100_BIAS_COMP_ACC_Y 0x1A
#define MAX21100_BIAS_COMP_ACC_Z 0x1B
#define MAX21100_FUS_CFG0        0x1C
#define MAX21100_FUS_CFG1        0x1D
#define MAX21100_YR_ODR_TRIM     0x1F





#define MAX21100_ADDRESS 0xB0   //  Device address of MAX21100 when ADO = 1
#define LIS3MDL_ADDRESS  0x38   //  Address of LIS3MDL magnetometer

//#define MAX21100_ADDRESS 0x58   //  Device address of MAX21100 when ADO = 0
//#define LIS3MDL_ADDRESS  0x1C   //  Address of LIS3MDL magnetometer



#define MAX21100Bypass false  // operate magnetometer as slave to MAX21100 when false

// Set initial input parameters
enum Ascale {
  AFS_16G = 0,
  AFS_8G,
  AFS_4G,
  AFS_2G
};

enum Aodr { // gyro Output Data Rate
  AODR_2kHz = 0,
  AODR_1kHz,
  AODR_500Hz,
  AODR_250Hz,
  AODR_125Hz,   // default
  AODR_62_5Hz,
  AODR_31_25Hz    //0x0F
};

enum Abw { // accel bandwidth
  ABW_div48 = 0,  // default, accel bandwidth is 1/48 of the accel ODR
  ABW_div22,
  ABW_div9,
  ABW_div3        // 0x03
};

enum Gscale {  // gyro full scale
  GFS_2000DPS = 0,
  GFS_1000DPS,
  GFS_500DPS,
  GFS_250DPS         // 0x03
};

enum Godr { // gyro Output Data Rate
  GODR_8kHz = 0,
  GODR_4kHz,
  GODR_2kHz,
  GODR_1kHz,
  GODR_500Hz,   // default
  GODR_250Hz,
  GODR_125Hz,
  GODR_62_5Hz,  // 62.5 Hz
  GODR_31_25Hz,
  GODR_15_625Hz,
  GODR_7_8125Hz,
  GODR_3_90625Hz

};

enum Gbw { // gyro bandwidth
  GBW_2Hz = 0,
  GBW_4Hz,
  GBW_6Hz,
  GBW_8Hz,
  GBW_10Hz,
  GBW_14Hz,
  GBW_22Hz,
  GBW_32Hz,
  GBW_50Hz,
  GBW_75Hz,
  GBW_100Hz,  // default = 0x0A
  GBW_150Hz,
  GBW_200Hz,
  GBW_250Hz,
  GBW_300Hz,
  GBW_400Hz  // 0x0F
};

enum powerMode {  // power modes without using DSYNC enable
  powerDownmode = 0, // 0x00
  gyrosleepmode,
  gyrospotmode,
  gyronormalmode,
  notused0,
  notused1,
  notused2,
  notused3,
  accelspotmode, // 0x08
  notused4,
  notused5,
  notused6,
  accelnormalmode,
  accelnormalgyrosleepmode,
  accelnormalgyrospotmode,
  accelnormalgyronormalmode  // 0x0F
};

enum Mscale {
  MFS_4Gauss = 0,  // 0.15 mG per LSB
  MFS_8Gauss,      // 0.30 mG per LSB
  MFS_12Gauss,     // 0.60 mG per LSB
  MFS_16Gauss      // 1.20 mG per LSB
};

enum Mopmode {
  MOM_lowpower = 0,
  MOM_medperf,
  MOM_hiperf,
  MOM_ultrahiperf
};

enum MSodr {        // magnetometer output data rate when slaved to the MAX21100
  MODR_div1 = 0,    // default, magnetometer ODR is 1/1 of the accel ODR
  MODR_div2,
  MODR_div4,
  MODR_div8,
  MODR_div16,
  MODR_div32,
  MODR_div64,
  MODR_div128
};

enum Modr {         // magnetometer output data rate MAX21100 is bypassed
  MODR_0_625Hz = 0,
  MODR_1_25Hz,
  MODR_2_5Hz,
  MODR_5Hz,
  MODR_10Hz,
  MODR_20Hz,
  MODR_40Hz,
  MODR_80Hz
};

uint8_t readByte(uint8_t address, uint8_t subAddress);
void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);
void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);

void initLIS3MDL();
void initbypassMAX21100();
void InitIMU();

typedef struct{
	float GyroX;
	float GyroY;
	float GyroZ;
	float AccelX;
	float AccelY;
	float AccelZ;
	float MagX;
	float MagY;
	float MagZ;
	float SensorTemp;
}SensdDat;

void ReadAllSensData(SensdDat *IMUData);
int16_t readMagTempData();

#endif /* SRC_IMU_H_ */
