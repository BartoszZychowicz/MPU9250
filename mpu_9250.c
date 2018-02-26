#include "stm32f10x.h"
#include "mpu_9250.h"
#include <stdio.h>
#include "delay.h"

float PI = 3.14159265358979323846f;
float GyroMeasError = 3.14159265358979323846f * (60.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
float beta = sqrt(3.0f / 4.0f) * (3.14159265358979323846f * (60.0f / 180.0f));  // compute beta
float GyroMeasDrift = 3.14159265358979323846f * (1.0f / 180.0f);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float zeta = sqrt(3.0f / 4.0f) * (3.14159265358979323846f * (1.0f / 180.0f));  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value


enum Ascale {
	AFS_2G = 0,
	AFS_4G,
	AFS_8G,
	AFS_16G
};

enum Gscale {
	GFS_250DPS = 0,
	GFS_500DPS,
	GFS_1000DPS,
	GFS_2000DPS
};

enum Mscale {
	MFS_14BITS = 0, // 0.6 mG per LSB
	MFS_16BITS      // 0.15 mG per LSB
};

uint8_t Ascale = AFS_2G;     // AFS_2G, AFS_4G, AFS_8G, AFS_16G
uint8_t Gscale = GFS_250DPS; // GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS
uint8_t Mscale = MFS_16BITS; // MFS_14BITS or MFS_16BITS, 14-bit or 16-bit magnetometer resolution
uint8_t Mmode = 0x06;        // Either 8 Hz 0x02) or 100 Hz (0x06) magnetometer data ODR
float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors



void getMres() {
	switch (Mscale)
	{
	// Possible magnetometer scales (and their register bit settings) are:
	// 14 bit resolution (0) and 16 bit resolution (1)
	case MFS_14BITS:
		mRes = 10.0*4912.0/8190.0; // Proper scale to return milliGauss
		break;
	case MFS_16BITS:
		mRes = 10.0*4912.0/32760.0; // Proper scale to return milliGauss
		break;
	}
}


void getGres() {
	switch (Gscale)
	{
	// Possible gyro scales (and their register bit settings) are:
	// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
	// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
	case GFS_250DPS:
		gRes = 250.0/32768.0;
		break;
	case GFS_500DPS:
		gRes = 500.0/32768.0;
		break;
	case GFS_1000DPS:
		gRes = 1000.0/32768.0;
		break;
	case GFS_2000DPS:
		gRes = 2000.0/32768.0;
		break;
	}
}


void getAres() {
	switch (Ascale)
	{
	// Possible accelerometer scales (and their register bit settings) are:
	// 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
	// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
	case AFS_2G:
		aRes = 2.0/32768.0;
		break;
	case AFS_4G:
		aRes = 4.0/32768.0;
		break;
	case AFS_8G:
		aRes = 8.0/32768.0;
		break;
	case AFS_16G:
		aRes = 16.0/32768.0;
		break;
	}
}

void mpu_set_reg(uint8_t device_addr , uint8_t reg)
{
	I2C_GenerateSTART(I2C1, ENABLE);
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS);
	I2C_Send7bitAddress(I2C1, device_addr, I2C_Direction_Transmitter);
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != SUCCESS);
	I2C_SendData(I2C1, reg);
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING) != SUCCESS);
}

void mpu_write(uint8_t device_addr, uint8_t reg, const void* data, int size)
{
	int i;
	const uint8_t* buffer = (uint8_t*)data;

	mpu_set_reg(device_addr, reg);
	for (i = 0; i < size; i++) {
		I2C_SendData(I2C1, buffer[i]);
		while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING) != SUCCESS);
	}
	I2C_GenerateSTOP(I2C1, ENABLE);
}

void mpu_read(uint8_t device_addr, uint8_t reg, void* data, int size)
{
	int i;
	uint8_t* buffer = (uint8_t*)data;
	mpu_set_reg(device_addr, reg);
	I2C_GenerateSTART(I2C1, ENABLE);
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS);
	I2C_AcknowledgeConfig(I2C1, ENABLE);
	I2C_Send7bitAddress(I2C1, device_addr, I2C_Direction_Receiver);
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) != SUCCESS);
	for (i = 0; i < size - 1; i++) {
		while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) != SUCCESS);
		buffer[i] = I2C_ReceiveData(I2C1);
	}
	I2C_AcknowledgeConfig(I2C1, DISABLE);
	I2C_GenerateSTOP(I2C1, ENABLE);
	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) != SUCCESS);
	buffer[i] = I2C_ReceiveData(I2C1);
}

void mpu_write_reg(uint8_t device_addr, uint8_t reg, uint8_t value)
{
	mpu_write(device_addr, reg, &value, sizeof(value));
}

uint8_t mpu_read_reg(uint8_t device_addr, uint8_t reg)
{
	uint8_t value = 0;
	mpu_read(device_addr, reg, &value, sizeof(value));
	return value;
}

int16_t mpu_read_value(uint8_t device_addr, uint8_t reg)
{
	int16_t value = 0;
	mpu_read(device_addr, reg, &value, sizeof(value));
	return value;
}

/*void mpu_read_multiple(uint8_t device_addr, uint8_t first_reg , uint8_t * destination , uint8_t count){
	int i;
	mpu_set_reg(device_addr, first_reg);
	I2C_GenerateSTART(I2C1, ENABLE);
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS);
	I2C_AcknowledgeConfig(I2C1, ENABLE);
	I2C_Send7bitAddress(I2C1, device_addr, I2C_Direction_Receiver);
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) != SUCCESS);
	for (i = 0; i < count - 1; i++) {
		while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) != SUCCESS);
		destination[i] = I2C_ReceiveData(I2C1);
	}
	I2C_AcknowledgeConfig(I2C1, DISABLE);
	I2C_GenerateSTOP(I2C1, ENABLE);
	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) != SUCCESS);
	destination[i] = I2C_ReceiveData(I2C1);
}*/

void mpu_reset(){
	mpu_write_reg(MPU9250_ADDR, PWR_MGMT_1, 0x80);
	delay_ms(10);
}

void mpu_ReadGyroData(int16_t * destination){
	uint8_t rawdata[6];

	mpu_read(MPU9250_ADDR, GYRO_XOUT_H, rawdata, 6);
	destination[0] = (int16_t)(((int16_t)rawdata[0] << 8) | rawdata[1]) ;
	destination[1] = (int16_t)(((int16_t)rawdata[2] << 8) | rawdata[3]) ;
	destination[2] = (int16_t)(((int16_t)rawdata[4] << 8) | rawdata[5]) ;
}

void mpu_ReadAccelData(int16_t * destination){
	uint8_t rawdata[6];

	mpu_read(MPU9250_ADDR, ACCEL_XOUT_H, rawdata, 6);
	destination[0] = (int16_t)(((int16_t)rawdata[0] << 8) | rawdata[1]) ;
	destination[1] = (int16_t)(((int16_t)rawdata[2] << 8) | rawdata[3]) ;
	destination[2] = (int16_t)(((int16_t)rawdata[4] << 8) | rawdata[5]) ;
}

void initAK8963(float * destination)
{
	// First extract the factory calibration for each magnetometer axis
	uint8_t rawData[3];  // x/y/z gyro calibration data stored here
	mpu_write_reg(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
	delay_ms(10);
	mpu_write_reg(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
	delay_ms(10);
	mpu_read(AK8963_ADDRESS, AK8963_ASAX, &rawData[0], 3);  // Read the x-, y-, and z-axis calibration values
	destination[0] =  (float)(rawData[0] - 128)/256.0f + 1.0f;   // Return x-axis sensitivity adjustment values, etc.
	destination[1] =  (float)(rawData[1] - 128)/256.0f + 1.0f;
	destination[2] =  (float)(rawData[2] - 128)/256.0f + 1.0f;
	mpu_write_reg(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
	delay_ms(10);
	// Configure the magnetometer for continuous read and highest resolution
	// set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
	// and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
	mpu_write_reg(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
	delay_ms(10);
}

void mpu_ReadMagData(int16_t * destination)
{
	uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
	if(mpu_read_reg(AK8963_ADDRESS, AK8963_ST1) & 0x01) { // wait for magnetometer data ready bit to be set
		mpu_read(AK8963_ADDRESS, AK8963_XOUT_L, &rawData[0], 7);  // Read the six raw data and ST2 registers sequentially into data array
		uint8_t c = rawData[6]; // End data read by reading ST2 register
		if(!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
			destination[0] = (int16_t)(((int16_t)rawData[1] << 8) | rawData[0]);  // Turn the MSB and LSB into a signed 16-bit value
			destination[1] = (int16_t)(((int16_t)rawData[3] << 8) | rawData[2]) ;  // Data stored as little Endian
			destination[2] = (int16_t)(((int16_t)rawData[5] << 8) | rawData[4]) ;
		}
	}
}

void mpu_initialize(){
	// wake up device
	// Clear sleep mode bit (6), enable all sensors
	mpu_write_reg(MPU9250_ADDR, PWR_MGMT_1, 0x00);
	delay_ms(100); // Wait for all registers to reset

	// Get stable time source
	// Auto select clock source to be PLL gyroscope reference if ready else
	mpu_write_reg(MPU9250_ADDR, PWR_MGMT_1, 0x01);
	delay_ms(200);

	// Configure Gyro and Thermometer
	// Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz,
	// respectively;
	// minimum delay_ms time for this setting is 5.9 ms, which means sensor fusion
	// update rates cannot be higher than 1 / 0.0059 = 170 Hz
	// DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
	// With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!),
	// 8 kHz, or 1 kHz
	mpu_write_reg(MPU9250_ADDR, CONFIG, 0x03);

	// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	// Use a 200 Hz rate; a rate consistent with the filter update rate
	// determined inset in CONFIG above.
	mpu_write_reg(MPU9250_ADDR, SMPLRT_DIV, 0x04);

	// Set gyroscope full scale range
	// Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are
	// left-shifted into positions 4:3

	// get current GYRO_CONFIG register value
	uint8_t c = mpu_read_reg(MPU9250_ADDR, GYRO_CONFIG);
	c = c & ~0xE0; // Clear self-test bits [7:5]
	c = c & ~0x02; // Clear Fchoice bits [1:0]
	c = c & ~0x18; // Clear AFS bits [4:3]
	c = c | Gscale << 3; // Set full scale range for the gyro
	// Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of
	// GYRO_CONFIG
	// c =| 0x00;
	// Write new GYRO_CONFIG value to register
	mpu_write_reg(MPU9250_ADDR, GYRO_CONFIG, c );

	// Set accelerometer full-scale range configuration
	// Get current ACCEL_CONFIG register value
	c = mpu_read_reg(MPU9250_ADDR, ACCEL_CONFIG);
	c = c & ~0xE0; // Clear self-test bits [7:5]
	c = c & ~0x18;  // Clear AFS bits [4:3]
	c = c | Ascale << 3; // Set full scale range for the accelerometer
	// Write new ACCEL_CONFIG register value
	mpu_write_reg(MPU9250_ADDR, ACCEL_CONFIG, c);

	// Set accelerometer sample rate configuration
	// It is possible to get a 4 kHz sample rate from the accelerometer by
	// choosing 1 for accel_fchoice_b bit [3]; in this case the bandwidth is
	// 1.13 kHz
	// Get current ACCEL_CONFIG2 register value
	c = mpu_read_reg(MPU9250_ADDR, ACCEL_CONFIG2);
	c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
	c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
	// Write new ACCEL_CONFIG2 register value
	mpu_write_reg(MPU9250_ADDR, ACCEL_CONFIG2, c);
	// The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
	// but all these rates are further reduced by a factor of 5 to 200 Hz because
	// of the SMPLRT_DIV setting

	//=================================INTERRUPTY==================================


	// Configure Interrupts and Bypass Enable
	// Set interrupt pin active high, push-pull, hold interrupt pin level HIGH
	// until interrupt cleared, clear on read of INT_STATUS, and enable
	// I2C_BYPASS_EN so additional chips can join the I2C bus and all can be
	// controlled by the Arduino as master.
	mpu_write_reg(MPU9250_ADDR, INT_PIN_CFG, 0x22);
	// Enable data ready (bit 0) interrupt
	mpu_write_reg(MPU9250_ADDR, INT_ENABLE, 0x01);
	delay_ms(100);

}

void mpu_calibrate(float * dest1, float * dest2)
{
	uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
	uint16_t ii, packet_count, fifo_count;
	int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

	// reset device, reset all registers, clear gyro and accelerometer bias registers
	mpu_write_reg(MPU9250_ADDR, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
	delay_ms(100);

	// get stable time source
	// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
	mpu_write_reg(MPU9250_ADDR, PWR_MGMT_1, 0x01);
	mpu_write_reg(MPU9250_ADDR, PWR_MGMT_2, 0x00);
	delay_ms(200);

	// Configure device for bias calculation
	mpu_write_reg(MPU9250_ADDR, INT_ENABLE, 0x00);   // Disable all interrupts
	mpu_write_reg(MPU9250_ADDR, FIFO_EN, 0x00);      // Disable FIFO
	mpu_write_reg(MPU9250_ADDR, PWR_MGMT_1, 0x00);   // Turn on internal clock source
	mpu_write_reg(MPU9250_ADDR, I2C_MST_CTRL, 0x00); // Disable I2C master
	mpu_write_reg(MPU9250_ADDR, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
	mpu_write_reg(MPU9250_ADDR, USER_CTRL, 0x0C);    // Reset FIFO and DMP
	delay_ms(15);

	// Configure MPU9250 gyro and accelerometer for bias calculation
	mpu_write_reg(MPU9250_ADDR, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
	mpu_write_reg(MPU9250_ADDR, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
	mpu_write_reg(MPU9250_ADDR, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
	mpu_write_reg(MPU9250_ADDR, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

	uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
	uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

	// Configure FIFO to capture accelerometer and gyro data for bias calculation
	mpu_write_reg(MPU9250_ADDR, USER_CTRL, 0x40);   // Enable FIFO
	mpu_write_reg(MPU9250_ADDR, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO (max size 512 bytes in MPU-9250)
	delay_ms(30); // accumulate 40 samples in 80 milliseconds = 480 bytes

	// At end of sample accumulation, turn off FIFO sensor read
	mpu_write_reg(MPU9250_ADDR, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
	mpu_read(MPU9250_ADDR, FIFO_COUNTH, &data[0] , 2); // read FIFO sample count
	fifo_count = ((uint16_t)data[0] << 8) | data[1];
	packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

	for (ii = 0; ii < packet_count; ii++) {
		int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
		mpu_read(MPU9250_ADDR, FIFO_R_W, &data[0], 12); // read data for averaging
		accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
		accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
		accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
		gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
		gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
		gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

		accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
		accel_bias[1] += (int32_t) accel_temp[1];
		accel_bias[2] += (int32_t) accel_temp[2];
		gyro_bias[0]  += (int32_t) gyro_temp[0];
		gyro_bias[1]  += (int32_t) gyro_temp[1];
		gyro_bias[2]  += (int32_t) gyro_temp[2];

	}
	accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
	accel_bias[1] /= (int32_t) packet_count;
	accel_bias[2] /= (int32_t) packet_count;
	gyro_bias[0]  /= (int32_t) packet_count;
	gyro_bias[1]  /= (int32_t) packet_count;
	gyro_bias[2]  /= (int32_t) packet_count;

	if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
	else {accel_bias[2] += (int32_t) accelsensitivity;}

	// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
	data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
	data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
	data[3] = (-gyro_bias[1]/4)       & 0xFF;
	data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
	data[5] = (-gyro_bias[2]/4)       & 0xFF;

	/// Push gyro biases to hardware registers
	/*  writeByte(MPU9250_ADDR, XG_OFFSET_H, data[0]);
  writeByte(MPU9250_ADDR, XG_OFFSET_L, data[1]);
  writeByte(MPU9250_ADDR, YG_OFFSET_H, data[2]);
  writeByte(MPU9250_ADDR, YG_OFFSET_L, data[3]);
  writeByte(MPU9250_ADDR, ZG_OFFSET_H, data[4]);
  writeByte(MPU9250_ADDR, ZG_OFFSET_L, data[5]);
	 */
	dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
	dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
	dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

	// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
	// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
	// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
	// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
	// the accelerometer biases calculated above must be divided by 8.

	int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
	mpu_read(MPU9250_ADDR, XA_OFFSET_H, &data[0], 2); // Read factory accelerometer trim values
	accel_bias_reg[0] = (int16_t) ((int16_t)data[0] << 8) | data[1];
	mpu_read(MPU9250_ADDR, YA_OFFSET_H, &data[0], 2);
	accel_bias_reg[1] = (int16_t) ((int16_t)data[0] << 8) | data[1];
	mpu_read(MPU9250_ADDR, ZA_OFFSET_H, &data[0], 2);
	accel_bias_reg[2] = (int16_t) ((int16_t)data[0] << 8) | data[1];

	uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
	uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

	for(ii = 0; ii < 3; ii++) {
		if(accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
	}

	// Construct total accelerometer bias, including calculated average accelerometer bias from above
	accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
	accel_bias_reg[1] -= (accel_bias[1]/8);
	accel_bias_reg[2] -= (accel_bias[2]/8);

	data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
	data[1] = (accel_bias_reg[0])      & 0xFF;
	data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
	data[3] = (accel_bias_reg[1])      & 0xFF;
	data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	data[5] = (accel_bias_reg[2])      & 0xFF;
	data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

	// Apparently this is not working for the acceleration biases in the MPU-9250
	// Are we handling the temperature correction bit properly?
	// Push accelerometer biases to hardware registers
	/*  writeByte(MPU9250_ADDR, XA_OFFSET_H, data[0]);
  writeByte(MPU9250_ADDR, XA_OFFSET_L, data[1]);
  writeByte(MPU9250_ADDR, YA_OFFSET_H, data[2]);
  writeByte(MPU9250_ADDR, YA_OFFSET_L, data[3]);
  writeByte(MPU9250_ADDR, ZA_OFFSET_H, data[4]);
  writeByte(MPU9250_ADDR, ZA_OFFSET_L, data[5]);
	 */
	// Output scaled accelerometer biases for manual subtraction in the main program
	dest2[0] = (float)accel_bias[0]/(float)accelsensitivity;
	dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
	dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
}


void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
	float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
	float norm;
	float hx, hy, _2bx, _2bz;
	float s1, s2, s3, s4;
	float qDot1, qDot2, qDot3, qDot4;

	// Auxiliary variables to avoid repeated arithmetic
	float _2q1mx;
	float _2q1my;
	float _2q1mz;
	float _2q2mx;
	float _4bx;
	float _4bz;
	float _2q1 = 2.0f * q1;
	float _2q2 = 2.0f * q2;
	float _2q3 = 2.0f * q3;
	float _2q4 = 2.0f * q4;
	float _2q1q3 = 2.0f * q1 * q3;
	float _2q3q4 = 2.0f * q3 * q4;
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q1q4 = q1 * q4;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q2q4 = q2 * q4;
	float q3q3 = q3 * q3;
	float q3q4 = q3 * q4;
	float q4q4 = q4 * q4;

	// Normalise accelerometer measurement
	norm = sqrt(ax * ax + ay * ay + az * az);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f/norm;
	ax *= norm;
	ay *= norm;
	az *= norm;

	// Normalise magnetometer measurement
	norm = sqrt(mx * mx + my * my + mz * mz);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f/norm;
	mx *= norm;
	my *= norm;
	mz *= norm;

	// Reference direction of Earth's magnetic field
	_2q1mx = 2.0f * q1 * mx;
	_2q1my = 2.0f * q1 * my;
	_2q1mz = 2.0f * q1 * mz;
	_2q2mx = 2.0f * q2 * mx;
	hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
	hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
	_2bx = sqrt(hx * hx + hy * hy);
	_2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
	_4bx = 2.0f * _2bx;
	_4bz = 2.0f * _2bz;

	// Gradient decent algorithm corrective step
	s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
	norm = 1.0f/norm;
	s1 *= norm;
	s2 *= norm;
	s3 *= norm;
	s4 *= norm;

	// Compute rate of change of quaternion
	qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
	qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
	qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
	qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

	// Integrate to yield quaternion
	q1 += qDot1 * deltat;
	q2 += qDot2 * deltat;
	q3 += qDot3 * deltat;
	q4 += qDot4 * deltat;
	norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
	norm = 1.0f/norm;
	q[0] = q1 * norm;
	q[1] = q2 * norm;
	q[2] = q3 * norm;
	q[3] = q4 * norm;

}

void magcalMPU9250(float * dest1, float * dest2)
{
	printf("starting!");
	uint16_t ii = 0, sample_count = 0;
	int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
	int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};

	printf("Mag Calibration: Wave device in a figure eight until done!");
	delay_ms(4000);

	// shoot for ~fifteen seconds of mag data
	if(Mmode == 0x02) sample_count = 128;  // at 8 Hz ODR, new mag data is available every 125 ms
	if(Mmode == 0x06) sample_count = 1500;  // at 100 Hz ODR, new mag data is available every 10 ms
	for(ii = 0; ii < sample_count; ii++) {
		mpu_ReadMagData(mag_temp);  // Read the mag data
		printf("%i %i %i \n", mag_temp[0], mag_temp[1], mag_temp[2]);
		for (int jj = 0; jj < 3; jj++) {
			if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
			if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
		}
		if(Mmode == 0x02) delay_ms(135);  // at 8 Hz ODR, new mag data is available every 125 ms
		if(Mmode == 0x06) delay_ms(12);  // at 100 Hz ODR, new mag data is available every 10 ms
	}


	// Get hard iron correction
	mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
	mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
	mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts
	printf("%i %i %i %i %i %i \n", mag_max[0], mag_min[0], mag_max[1], mag_min[1], mag_max[2], mag_min[2] );
	printf("%i %i %i \n", mag_bias[0], mag_bias[1], mag_bias[2]);
	printf("%f \n",mRes);
	printf("%f %f %f\n", magCalibration[0], magCalibration[1], magCalibration[2] );

	dest1[0] = (float) mag_bias[0]*mRes*magCalibration[0];  // save mag biases in G for main program
	dest1[1] = (float) mag_bias[1]*mRes*magCalibration[1];
	dest1[2] = (float) mag_bias[2]*mRes*magCalibration[2];

	// Get soft iron correction estimate
	mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
	mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
	mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

	float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
	avg_rad /= 3.0;

	dest2[0] = avg_rad/((float)mag_scale[0]);
	dest2[1] = avg_rad/((float)mag_scale[1]);
	dest2[2] = avg_rad/((float)mag_scale[2]);

	printf("Mag Calibration done!");
	printf("%f %f %f \n", dest1[0], dest1[1], dest1[2]);
	printf("%f %f %f \n", dest2[0], dest2[1], dest2[2]);
}




//======================SELFTEST_nie_dziala========================

/*

for(int ii=0; ii<AVGSAMPLES ; ii++)
	{
		mpu_ReadAccelData(temp);


		//obliczanie sredniej
		for (int i = 0; i < 3; i++){
			aAvg[i] += temp[i] / AVGSAMPLES;
			rest = temp[i] % AVGSAMPLES;
			if (avg_temp_accel[i] >= (AVGSAMPLES - rest)){
				aAvg[i]++;
				avg_temp_accel[i] -= (AVGSAMPLES - rest);
			}
			else{
				avg_temp_accel[i]+=rest;
			}
		}


		mpu_ReadGyroData(temp);

		for (int i = 0; i < 3; i++){
			gAvg[i] += temp[i] / AVGSAMPLES;
			rest = temp[i] % AVGSAMPLES;
			if (avg_temp_gyro[i] >= (AVGSAMPLES - rest)){
				gAvg[i]++;
				avg_temp_gyro[i] -= (AVGSAMPLES - rest);
			}
			else{
				avg_temp_gyro[i]+=rest;
			}
		}
	}

	for(int i = 0; i<3; i++){
		aAvg[i] += avg_temp_accel[i];
		gAvg[i] += avg_temp_gyro[i];
	}

	printf("Average accel x: %i\n" , aAvg[0]);
	printf("Average accel y: %i\n" , aAvg[1]);
	printf("Average accel z: %i\n" , aAvg[2]);
	printf("Average gyro x: %i\n" , gAvg[0]);
	printf("Average gyro y: %i\n" , gAvg[1]);
	printf("Average gyro z: %i\n" , gAvg[2]);


	delay_ms(1000);
	mpu_write_reg(MPU9250_ADDR, ACCEL_CONFIG, 0xE0);
	mpu_write_reg(MPU9250_ADDR, GYRO_CONFIG, 0xE0);
	delay_ms(25);

	for(int ii=0; ii<AVGSAMPLES ; ii++)
	{
		mpu_ReadAccelData(temp);
		printf("wartosc: %i\n" , temp[2]);
		for (int i = 0; i < 3; i++){
			aSTAvg[i] += temp[i] / AVGSAMPLES;
			rest = temp[i] % AVGSAMPLES;
			if (avg_temp_accel[i] >= (AVGSAMPLES - rest)){
				aSTAvg[i]++;
				avg_temp_accel[i] -= (AVGSAMPLES - rest);
			}
			else{
				avg_temp_accel[i]+=rest;
			}
		}
		printf("srednia: %i\n" , aSTAvg[2]);
		mpu_ReadGyroData(temp);

		for (int i = 0; i < 3; i++){
			gSTAvg[i] += temp[i] / AVGSAMPLES;
			rest = temp[i] % AVGSAMPLES;
			if (avg_temp_gyro[i] >= (AVGSAMPLES - rest)){
				gSTAvg[i]++;
				avg_temp_gyro[i] -= (AVGSAMPLES - rest);
			}
			else{
				avg_temp_gyro[i]+=rest;
			}
		}
	}

	for(int i = 0; i<3; i++){
		aSTAvg[i] += avg_temp_accel[i];
		gSTAvg[i] += avg_temp_gyro[i];
	}

	printf("Selftest accel x: %i\n" , aSTAvg[0]);
	printf("Selftest accel y: %i\n" , aSTAvg[1]);
	printf("Selftest accel z: %i\n" , aSTAvg[2]);
	printf("Selftest gyro x: %i\n" , gSTAvg[0]);
	printf("Selftest gyro y: %i\n" , gSTAvg[1]);
	printf("Selftest gyro z: %i\n" , gSTAvg[2]);


	mpu_write_reg(MPU9250_ADDR, ACCEL_CONFIG, 0x00);
	mpu_write_reg(MPU9250_ADDR, GYRO_CONFIG, 0x00);
	delay_ms(25); // Delay a while to let the device stabilize

	// Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
	selfTest[0] = mpu_read_reg(MPU9250_ADDR, SELF_TEST_X_ACCEL); // X-axis accel self-test results
	printf("selftest accel x: %i\n", selfTest[0]);
	selfTest[1] = mpu_read_reg(MPU9250_ADDR, SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
	printf("selftest accel y: %i\n", selfTest[1]);
	selfTest[2] = mpu_read_reg(MPU9250_ADDR, SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
	printf("selftest accel z: %i\n", selfTest[2]);
	selfTest[3] = mpu_read_reg(MPU9250_ADDR, SELF_TEST_X_GYRO); // X-axis gyro self-test results
	printf("selftest gyro x: %i\n", selfTest[3]);
	selfTest[4] = mpu_read_reg(MPU9250_ADDR, SELF_TEST_Y_GYRO); // Y-axis gyro self-test results
	printf("selftest gyro y: %i\n", selfTest[4]);
	selfTest[5] = mpu_read_reg(MPU9250_ADDR, SELF_TEST_Z_GYRO); // Z-axis gyro self-test results
	printf("selftest gyro z: %i\n", selfTest[5]);

	// Retrieve factory self-test value from self-test code reads
	factoryTrim[0] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[0] - 1.0) )); // FT[Xa] factory trim calculation
	printf("%f\n", factoryTrim[0]);
	factoryTrim[1] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[1] - 1.0) )); // FT[Ya] factory trim calculation
	printf("%f\n", factoryTrim[1]);
	factoryTrim[2] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[2] - 1.0) )); // FT[Za] factory trim calculation
	printf("%f\n", factoryTrim[2]);
	factoryTrim[3] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
	printf("%f\n", factoryTrim[3]);
	factoryTrim[4] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
	printf("%f\n", factoryTrim[4]);
	factoryTrim[5] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation
	printf("%f\n", factoryTrim[5]);

	// Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
	// To get percent, must multiply by 100
	for (int i = 0; i < 3; i++) {

		ResultST[i] = 100*((float)(aSTAvg[i] - aAvg[i]))/factoryTrim[i]; // Report percent differences
		ResultST[i+3] = 100*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3]; // Report percent differences
	}

	printf("Selftest gyro x: %f, y: %f , z: %f\n" , ResultST[0], ResultST[1], ResultST[2]);
	printf("Selftest accel x: %f, y: %f , z: %f\n" , ResultST[3], ResultST[4], ResultST[5]);



 */
