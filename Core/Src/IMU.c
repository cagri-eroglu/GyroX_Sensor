
#include "IMU.h"
#include <math.h>
extern I2C_HandleTypeDef hi2c1;


// Specify sensor full scale
uint8_t Gscale = GFS_250DPS;  // Gyro full scale
uint8_t Godr = GODR_250Hz;    // Gyro sample rate
uint8_t Gbw = GBW_22Hz;       // Gyro bandwidth
uint8_t Ascale = AFS_2G;      // Accel full scale
uint8_t Aodr = AODR_250Hz;    // Accel sample rate
uint8_t Abw = ABW_div9;       // Accel bandwidth, accel sample rate divided by ABW_divx
uint8_t Mscale = MFS_4Gauss;  // Select magnetometer full-scale resolution
uint8_t Mopmode = MOM_hiperf; // Select magnetometer perfomance mode
uint8_t Modr = MODR_10Hz;     // Select magnetometer ODR when in MAX21100 bypass mode
uint8_t MSodr = MODR_div16;   // Select magnetometer ODR as Aodr/MODR_divx
uint8_t powerSelect = 0x00;   // no DSYNC enable or usage
uint8_t powerMode = accelnormalgyronormalmode;  // specify power mode for accel + gyro
uint8_t status;               // MAX21100 data status register
float aRes, gRes, mRes;       // scale resolutions per LSB for the sensors


int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
float magCalibration[3] = {0, 0, 0}, magbias[3] = {0, 0, 0};  // Factory mag calibration and mag bias
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0};      // Bias corrections for gyro and accelerometer
int16_t tempGCount, tempMCount;      // temperature raw count output of mag and gyro
float   Gtemperature, Mtemperature; // Stores the MAX21100 gyro and LIS3MDL mag internal chip temperatures in degrees Celsius
double Temperature, Pressure;       // stores MS5637 pressures sensor pressure and temperature
float SelfTest[12];                  // holds results of gyro and accelerometer self test




float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values




void InitIMU(){

	if(HAL_I2C_IsDeviceReady(&hi2c1, MAX21100_ADDRESS, 3, 0xff)==HAL_OK){
		if(readByte(MAX21100_ADDRESS, MAX21100_WHO_AM_I)==0xb2){

		}
			//trcCDCTransmitStr("Gyro and Accel Ready\n");

	}

	if(HAL_I2C_IsDeviceReady(&hi2c1, LIS3MDL_ADDRESS, 3, 0xff)==HAL_OK){
		if(readByte(LIS3MDL_ADDRESS, LIS3MDL_WHO_AM_I)==0x3d){

		}
			//trcCDCTransmitStr("Magnetometer Ready\n");

	}
	initLIS3MDL();
	initbypassMAX21100();
	gRes = 250.0f/32768.0f;
	aRes = 2.0f/32768.0f;
	mRes = 4.0f/32.768f; // Proper scale to return milliGauss
}


//===================================================================================================================
//====== Set of useful function to access acceleration. gyroscope, magnetometer, and temperature data
//===================================================================================================================


void readAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(MAX21100_ADDRESS, MAX21100_ACC_X_H, 2, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;      // Turn the MSB and LSB into a signed 16-bit value
  readBytes(MAX21100_ADDRESS, MAX21100_ACC_Y_H, 2, &rawData[0]);
  destination[1] = ((int16_t)rawData[0] << 8) | rawData[1] ;
  readBytes(MAX21100_ADDRESS, MAX21100_ACC_Z_H, 2, &rawData[0]);
  destination[2] = ((int16_t)rawData[0] << 8) | rawData[1] ;
}


void readGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(MAX21100_ADDRESS, MAX21100_GYRO_X_H, 2, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;       // Turn the MSB and LSB into a signed 16-bit value
  readBytes(MAX21100_ADDRESS, MAX21100_GYRO_Y_H, 2, &rawData[0]);
  destination[1] = ((int16_t)rawData[0] << 8) | rawData[1] ;
  readBytes(MAX21100_ADDRESS, MAX21100_GYRO_Z_H, 2, &rawData[0]);
  destination[2] = ((int16_t)rawData[0] << 8) | rawData[1] ;
}

int16_t readGyroTempData()
{
  uint8_t rawData[2];  // x/y/z gyro register data stored here
  readBytes(MAX21100_ADDRESS, MAX21100_TEMP_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
  return ((int16_t)rawData[0] << 8) | rawData[1] ;               // Turn the MSB and LSB into a 16-bit value
}

void readMagData(int16_t * destination)
{

  uint8_t rawData[2];  // x/y/z mag register data, little Endian
  readBytes(LIS3MDL_ADDRESS, LIS3MDL_OUT_X_L, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
    destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
  readBytes(LIS3MDL_ADDRESS, LIS3MDL_OUT_Y_L, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
    destination[1] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
  readBytes(LIS3MDL_ADDRESS, LIS3MDL_OUT_Z_L, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
    destination[2] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value

}

void ReadAllSensData(SensdDat *IMUData){

int16_t accelCount[3],gyroCount[3],magCount[3],tempGCount;
uint8_t status = 0;
status = readByte(MAX21100_ADDRESS, MAX21100_SYSTEM_STATUS);
if((status & 0x04)==0x04 ){
   readAccelData(accelCount);  // Read the x/y/z adc values
   // Now we'll calculate the accleration value into actual g's
   IMUData->AccelX = (float)accelCount[0]*aRes;
   IMUData->AccelY = (float)accelCount[1]*aRes;
   IMUData->AccelZ= (float)accelCount[2]*aRes;
}

if((status & 0x01)==0x01 ){
   readGyroData(gyroCount);  // Read the x/y/z gyro
   // Calculate the gyro value into actual degrees per second
   IMUData->GyroX = (float)gyroCount[0]*gRes; // - gyroBias[0];  // get actual gyro value, this depends on scale being set
   IMUData->GyroY = (float)gyroCount[1]*gRes; // - gyroBias[1];
   IMUData->GyroZ = (float)gyroCount[2]*gRes; // - gyroBias[2];
}


   readMagData(magCount);  // Read the x/y/z adc values
   // Calculate the magnetometer values in milliGauss
   IMUData->MagX = (float)magCount[0]*mRes; //  get actual magnetometer value, this depends on scale being set
   IMUData->MagY = (float)magCount[1]*mRes; //
   IMUData->MagZ = (float)magCount[2]*mRes; //
   tempGCount=readMagTempData();
   IMUData->SensorTemp=((float) tempGCount) ;

}

int16_t readMagTempData()
{
  uint8_t rawData[2];  // x/y/z gyro register data stored here
  readBytes(LIS3MDL_ADDRESS, LIS3MDL_TEMP_OUT_L, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
  return ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a 16-bit value
}


void initLIS3MDL() {
// Configure magnetometer
// Choose device mode (bits 1:0 = 00 = continuous data read, 01 = single conversion, 10 & 11 = default power down)
writeByte(LIS3MDL_ADDRESS, LIS3MDL_CTRL_REG3, 0x00); // Enable continuous data read mode (bits 1:0 = 00)
// Enable temperature sensor (bit 7 = 1)
// Set magnetometer operative mode for x and y axes (bits 6:5)
// Set magnetometer ODR (bits 4:2)
writeByte(LIS3MDL_ADDRESS, LIS3MDL_CTRL_REG1, 0x80 | Mopmode << 5 | Modr << 2);
writeByte(LIS3MDL_ADDRESS, LIS3MDL_CTRL_REG2, Mscale << 5);  // Set magnetometer full scale range
writeByte(LIS3MDL_ADDRESS, LIS3MDL_CTRL_REG4, Mopmode << 2);  // Set magnetometer operative mode for z axis
writeByte(LIS3MDL_ADDRESS, LIS3MDL_CTRL_REG5, 0x40); // output registers not updated until both data bytes have been read
}



// Calibrate the MAX21100 gyro and accelerometer
void calibrateMAX21100(float * dest1, float * dest2) {

  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii = 0, fifo_count = 0, sample_count = 0;
  int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

  // Choose normal power mode (accelnormalgyronormalmode = 0x0F in bits 6:3
  // Enable all axes (z = bit 2, y = bit 1, x = bit 0)
   writeByte(MAX21100_ADDRESS, MAX21100_POWER_CFG, accelnormalgyronormalmode << 3 | 0x07);
   HAL_Delay(100);

// Configure gyro
  // Select 14 Hz gyro bandwidth (bits 5:2) and 250 dps gyro full scale (bits 1:0)
   writeByte(MAX21100_ADDRESS, MAX21100_GYRO_CFG1, GBW_14Hz << 2 | GFS_250DPS);
  // Select 125 Hz gyro ODR (bits 1:0)
   writeByte(MAX21100_ADDRESS, MAX21100_GYRO_CFG2, GODR_125Hz);
   HAL_Delay(100);

// Configure the accelerometer
// Select accel full scale (bits 7:6) and enable all three axes (bits 2:0)
   writeByte(MAX21100_ADDRESS, MAX21100_PWR_ACC_CFG, AFS_2G << 6 | 0x07);
  // Select 14 Hz accel band width (bits 5:4) and 125 Hz accel ODR (bits 3:0)
   writeByte(MAX21100_ADDRESS, MAX21100_ACC_CFG1, ABW_div9 << 4 | AODR_125Hz);
   HAL_Delay(100);

  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec per data sheet
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g per data sheet

   // 128 bytes in the FIFO = 64 words; use 63 words to collect 63/3 axes = 21 3-axis gyro data samples
   // Use FIFO to collect and average 21 gyro data samples
   writeByte(MAX21100_ADDRESS, MAX21100_FIFO_TH,  0x3C); // Set word threshold to 63 = 21/125 Hz = 168 ms of data
   writeByte(MAX21100_ADDRESS, MAX21100_FIFO_CFG, 0x41); // Set FIFO normal mode and accumulate gyro data
   HAL_Delay(200);  // Wait to collect gyro data in the FIFO

   if(readByte(MAX21100_ADDRESS, MAX21100_FIFO_STATUS) & 0x04) {  // Verify FIFO threshold reached

   fifo_count = readByte(MAX21100_ADDRESS, MAX21100_FIFO_COUNT);  // get number of samples in the FIFO
   sample_count = fifo_count/3; // should be 21 for 63 word fifo count
   for(ii = 0; ii < sample_count; ii++) {
    int16_t gyro_temp[3] = {0, 0, 0};
    readBytes(MAX21100_ADDRESS, MAX21100_FIFO_DATA, 6, &data[0]);  // Read the six FIFO registers per sample
//    readBytes(MAX21100_ADDRESS, MAX21100_GYRO_X_H, 6, &data[0]);  // Read the six FIFO registers per sample
    gyro_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]) ; // Form signed 16-bit integer for each sample in FIFO
    gyro_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]) ;
    gyro_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]) ;

    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];
   }
   }
    gyro_bias[0]  /= (int32_t) sample_count;  // get average gyro bias
    gyro_bias[1]  /= (int32_t) sample_count;
    gyro_bias[2]  /= (int32_t) sample_count;

  // Output scaled gyro biases for display in the main program
  dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity;
  dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
  dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

   // Now do the same for the accelerometer
   // 128 bytes in the FIFO = 64 words; use 63 words to collect 63/3 axes = 21 3-axis accel data samples
   // Use FIFO to collect and average 21 accel data samples
   writeByte(MAX21100_ADDRESS, MAX21100_FIFO_TH,  0x3C); // Set word threshold to 63 = 21/125 Hz = 168 ms of data
   writeByte(MAX21100_ADDRESS, MAX21100_FIFO_CFG, 0x42); // Set FIFO normal mode and accumulate accel data
   HAL_Delay(200);  // Wait to collect accel data in the FIFO

   if(readByte(MAX21100_ADDRESS, MAX21100_FIFO_STATUS) & 0x04) {  // Verify FIFO threshold reached

   fifo_count = readByte(MAX21100_ADDRESS, MAX21100_FIFO_COUNT);  // get number of samples in the FIFO
   sample_count = fifo_count/3;
   for(ii = 0; ii < sample_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0};
    readBytes(MAX21100_ADDRESS, MAX21100_FIFO_DATA, 6, &data[0]);  // Read the six FIFO registers per sample
//    readBytes(MAX21100_ADDRESS, MAX21100_ACC_X_H, 6, &data[0]);  // Read the six FIFO registers per sample
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]) ; // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]) ;
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]) ;

    accel_bias[0]  += (int32_t) accel_temp[0];
    accel_bias[1]  += (int32_t) accel_temp[1];
    accel_bias[2]  += (int32_t) accel_temp[2];
   }
   }
    accel_bias[0]  /= (int32_t) sample_count;  // get average gyro bias
    accel_bias[1]  /= (int32_t) sample_count;
    accel_bias[2]  /= (int32_t) sample_count;

  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) accelsensitivity;}

  // Output scaled gyro biases for display in the main program
  dest2[0] = (float) accel_bias[0]/(float) accelsensitivity;
  dest2[1] = (float) accel_bias[1]/(float) accelsensitivity;
  dest2[2] = (float) accel_bias[2]/(float) accelsensitivity;

   writeByte(MAX21100_ADDRESS, MAX21100_FIFO_CFG, 0x00); // Turn off FIFO

   writeByte(MAX21100_ADDRESS, MAX21100_BANK_SELECT, 0x02);  // select bank 2

   gyro_bias[0] *= -1;
   gyro_bias[1] *= -1;
   gyro_bias[2] *= -1;

   writeByte(MAX21100_ADDRESS, MAX21100_BIAS_GYRO_X_H, ((gyro_bias[0] >> 8) & 0x1F));  // load bias into gyro bias registers
   writeByte(MAX21100_ADDRESS, MAX21100_BIAS_GYRO_X_L, (gyro_bias[0]  & 0xFF));
   writeByte(MAX21100_ADDRESS, MAX21100_BIAS_GYRO_Y_H, ((gyro_bias[1] >> 8) & 0x1F));  // load bias into gyro bias registers
   writeByte(MAX21100_ADDRESS, MAX21100_BIAS_GYRO_Y_L, (gyro_bias[1]  & 0xFF));
   writeByte(MAX21100_ADDRESS, MAX21100_BIAS_GYRO_Z_H, ((gyro_bias[2] >> 8) & 0x1F));  // load bias into gyro bias registers
   writeByte(MAX21100_ADDRESS, MAX21100_BIAS_GYRO_Z_L, (gyro_bias[2]  & 0xFF));

   writeByte(MAX21100_ADDRESS, MAX21100_BANK_SELECT, 0x00);  // select bank 0
}


// Initialize the MAX21100 for bypass mode operations (read from magnetometer directly via microcontroller
void initbypassMAX21100() {
	writeByte(MAX21100_ADDRESS, MAX21100_POWER_CFG, 0x07);
  // Enable power select to control power mode from DSYNC (enable = 0x80, disable = 0x00)
  // choose power mode (accelnormal_gyronormal = 0xFF in bits 6:3
  // Enable all axes (z = bit 2, y = bit 1, x = bit 0)
   writeByte(MAX21100_ADDRESS, MAX21100_POWER_CFG, powerSelect | powerMode << 3 | 0x07);
   writeByte(MAX21100_ADDRESS, MAX21100_POWER_CFG, 0x7f);
// Configure gyro
  // Select gyro bandwidth (bits 5:2) and gyro full scale (bits 1:0)
   writeByte(MAX21100_ADDRESS, MAX21100_GYRO_CFG1, Gbw << 2 | Gscale);
  // Select gyro ODR (bits 1:0)
   writeByte(MAX21100_ADDRESS, MAX21100_GYRO_CFG2, Godr);

// Configure the accelerometer
// Select accel full scale (bits 7:6) and enable all three axes (bits 2:0)
   writeByte(MAX21100_ADDRESS, MAX21100_PWR_ACC_CFG, Ascale << 6 | 0x07);
  // Select accel band width (bits 5:4) and accel ODR (bits 3:0)
   writeByte(MAX21100_ADDRESS, MAX21100_ACC_CFG1, Abw << 4 | Aodr);

  // Data ready configuration
  // Enable bypass mode to read magnetometer directly from microcontroller (bit 7 = 1)
  // Clear data ready bits when status register is read (bits 3:2 = 10)
  // Enable fine temperature mode, enable temperature sensor (bits 1:0 = 01)
  writeByte(MAX21100_ADDRESS, MAX21100_DR_CFG, 0x80 | 0x08 | 0x01);
}



// I2C read/write functions for the MAX21100 and LIS3MDL sensors
void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
    uint8_t dataR[2];
    dataR[0]=subAddress;
    dataR[1]=data;
	//HAL_I2C_Master_Transmit(&hi2c1, address, dataR, 2, 0xff);
	HAL_I2C_Mem_Write(&hi2c1, address, subAddress, 1, &data, 1, 0xff);


}

uint8_t readByte(uint8_t address, uint8_t subAddress)
{
	uint8_t dataR;
	HAL_I2C_Mem_Read(&hi2c1, address, subAddress, 1, &dataR, 1, 0xff);

	return dataR;
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{
	HAL_I2C_Mem_Read(&hi2c1, address, subAddress, 1, dest, count, 0xff);
}

