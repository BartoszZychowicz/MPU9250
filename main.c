#include <math.h>
#include "stm32f10x.h"
#include "mpu_9250.h"
#include "delay.h"
#include "USART_lib.h"

#define AVGSAMPLES 10


// ZMIENNE GLOBALNE
float pitch, yaw, roll;
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
float deltat = 0.0f;                             // integration interval for both filter schemes
float magCalibration[3] = {0.0};
volatile char I2CWatchDog = 0;
uint8_t I2CResult = 0;
uint8_t I2CErrorCount = 0;

void TIM3_IRQHandler()
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
		I2CWatchDog = 1;
	}
}

void usart_initialize(){			//ENABLE UART i przerwania
	USART_InitTypeDef uart;
	USART_StructInit(&uart);
	uart.USART_BaudRate = 115200;
	USART_Init(USART2, &uart);
	USART_Cmd(USART2, ENABLE);
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
}

void nvic_initialize(){
	NVIC_InitTypeDef nvic;
	nvic.NVIC_IRQChannel = USART2_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority = 0;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);

	nvic.NVIC_IRQChannel = TIM3_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 1;
	nvic.NVIC_IRQChannelSubPriority = 1;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
}

void gpio_initialize(){
	GPIO_InitTypeDef gpio;

	GPIO_StructInit(&gpio);
	gpio.GPIO_Pin = GPIO_Pin_2;
	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &gpio);

	gpio.GPIO_Pin = GPIO_Pin_3;
	gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &gpio);

	gpio.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7; // SCL, SDA
	gpio.GPIO_Mode = GPIO_Mode_AF_OD;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &gpio);
}

void tim2_initialize(){
	TIM_TimeBaseInitTypeDef tim;
	TIM_TimeBaseStructInit(&tim);
	tim.TIM_CounterMode = TIM_CounterMode_Up;
	tim.TIM_Prescaler = 6400 - 1;
	TIM_TimeBaseInit(TIM2, &tim);
	TIM_Cmd(TIM2, ENABLE);
}

void tim3_initialize(){					//timeout I2C
	TIM_TimeBaseInitTypeDef tim;
	TIM_TimeBaseStructInit(&tim);
	tim.TIM_CounterMode = TIM_CounterMode_Up;
	tim.TIM_Prescaler = 64000 - 1;
	tim.TIM_Period = 50;
	TIM_TimeBaseInit(TIM3, &tim);
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
}

void i2c_initialize(){
	I2C_InitTypeDef i2c;
	I2C_StructInit(&i2c);
	i2c.I2C_Mode = I2C_Mode_I2C;
	i2c.I2C_ClockSpeed = 100000;
	I2C_Init(I2C1, &i2c);
	I2C_Cmd(I2C1, ENABLE);
}

void stm_initialize(){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	gpio_initialize();
	usart_initialize();
	nvic_initialize();
	tim2_initialize();
	tim3_initialize();
	i2c_initialize();

	SysTick_Config(SystemCoreClock / 1000);
}

int main(void)
{
	stm_initialize();

	// =================NIEUZYWANE====================

	//int16_t gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
	//int16_t avg_temp_gyro[3], avg_temp_accel[3];
	//uint8_t selfTest[6];
	//float factoryTrim[6] = {0.0};
	//float ResultST[6] = {0.0};
	//float hardiron_correction[3] = {0.0};
	//float softiron_correction[3] = {0.0};
	//uint8_t FS = 0;

	//================================================

	int16_t temp[3];
	int counter_wysw = 0;

	float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0}, magbias[3] = {0, 0, 0};

	int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
	int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
	int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
	float ax, ay, az, gx, gy, gz, mx, my, mz;
	int t = 0;

	//ZMIENNE ENKODERA
	float angleValue = 0;
	uint8_t result = 0;


	// =======================TEST KOMUNIKACJI===========================

	printf("Wyszukiwanie akcelerometru...\n");

	uint8_t who_am_i = I2CReadReg(MPU9250_ADDR, WHO_AM_I);
	if (who_am_i == 0x71) {
		printf("Znaleziono akcelerometr MPU9250!\n");
	}
	else {
		printf("Niepoprawna odpowiedz ukladu(0x%02X)\n", who_am_i);
	}

	// ============================RESET MPU=============================

	printf("Reset MPU9250 \n");
	mpu_reset();

	// ===========================KALIBRACJA MPU==========================

	mpu_calibrate(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
	printf("x gyro bias = %f\n\r", gyroBias[0]);
	printf("y gyro bias = %f\n\r", gyroBias[1]);
	printf("z gyro bias = %f\n\r", gyroBias[2]);
	printf("x accel bias = %f\n\r", accelBias[0]);
	printf("y accel bias = %f\n\r", accelBias[1]);
	printf("z accel bias = %f\n\r", accelBias[2]);
	delay_ms(2000);

	//

	mpu_initialize();

	//========================================================================

	initAK8963(magCalibration);

	printf("%f %f %f", magCalibration[0], magCalibration[1], magCalibration[2]);
	printf("AK8963 initialized for active data mode....\n\r"); // Initialize device for active mode read of magnetometer
	printf("Accelerometer full-scale range = %f  g\n\r", 2.0f*(float)(1<<Ascale));
	printf("Gyroscope full-scale range = %f  deg/s\n\r", 250.0f*(float)(1<<Gscale));
	if(Mscale == 0) printf("Magnetometer resolution = 14  bits\n\r");
	if(Mscale == 1) printf("Magnetometer resolution = 16  bits\n\r");
	if(Mmode == 2) printf("Magnetometer ODR = 8 Hz\n\r");
	if(Mmode == 6) printf("Magnetometer ODR = 100 Hz\n\r");

	getAres(); // Get accelerometer sensitivity
	getGres(); // Get gyro sensitivity
	getMres(); // Get magnetometer sensitivity

	//magcalMPU9250(hardiron_correction, softiron_correction);

	mpu_ReadAccelData(temp);
	printf("Accel: %i %i %i\n", temp[0], temp[1], temp[2]);
	delay_ms(50);
	mpu_ReadGyroData(temp);
	printf("Gyro: %i %i %i \n", temp[0], temp[1], temp[2]);
	delay_ms(50);
	mpu_ReadMagData(temp);
	printf("Mag: %i %i %i\n", temp[0], temp[1], temp[2]);

	magbias[0] = -73.48;  // User environmental x-axis correction in milliGauss, should be automatically calculated
	magbias[1] = 526.92;  // User environmental y-axis correction in milliGauss
	magbias[2] = -425.04;  // User environmental z-axis correction in milliGauss

	while(1) {
		// If intPin goes high, all data registers have new data
		if(I2CReadReg(MPU9250_ADDR, INT_STATUS) & 0x01) {  // On interrupt, check if data ready interrupt

			mpu_ReadAccelData(accelCount);  // Read the x/y/z adc values
			// Now we'll calculate the acceleration value into actual g's
			ax = (float)accelCount[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
			ay = (float)accelCount[1]*aRes - accelBias[1];
			az = (float)accelCount[2]*aRes - accelBias[2];

			mpu_ReadGyroData(gyroCount);  // Read the x/y/z adc values
			// Calculate the gyro value into actual degrees per second
			gx = (float)gyroCount[0]*gRes - gyroBias[0];  // get actual gyro value, this depends on scale being set
			gy = (float)gyroCount[1]*gRes - gyroBias[1];
			gz = (float)gyroCount[2]*gRes - gyroBias[2];

			mpu_ReadMagData(magCount);  // Read the x/y/z adc values
			// Calculate the magnetometer values in milliGauss
			// Include factory calibration per data sheet and user environmental corrections

			mx = (float)magCount[0]*mRes*magCalibration[0] - magbias[0];  // get actual magnetometer value, this depends on scale being set
			my = (float)magCount[1]*mRes*magCalibration[1] - magbias[1];
			mz = (float)magCount[2]*mRes*magCalibration[2] - magbias[2];

			TIM_Cmd(TIM2, DISABLE);	//czas cyklu dla MPU
			t = TIM_GetCounter(TIM2);
			deltat = (float)t/10000.0;

			MadgwickQuaternionUpdate(-ay, -ax, az, gy*PI/180.0f, gx*PI/180.0f, -gz*PI/180.0f,  mx,  my, mz);

			TIM_SetCounter(TIM2, 0);
			TIM_Cmd(TIM2, ENABLE);	//odliczanie czasu cyklu dla MPU

			yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
			pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
			roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
			pitch *= 180.0f / PI;
			yaw   *= 180.0f / PI;
			yaw   += 5.46f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
			roll  *= 180.0f / PI;

			counter_wysw++;
			if (counter_wysw >= 100){
				printf("Accel values: %i %i %i \n", accelCount[0], accelCount[1], accelCount[2]);
				printf("Ax, Ay, Az values: %f %f %f \n", ax, ay, az);
				printf("Gyro values: %i %i %i \n", gyroCount[0], gyroCount[1], gyroCount[2]);
				printf("Gx, Gy, Gz values: %f %f %f \n", gx, gy, gz);
				printf("Mag values: %i %i %i \n", magCount[0], magCount[1], magCount[2]);
				printf("mx, my, mz values: %f %f %f \n\n", mx, my, mz);
				printf("Yaw, Pitch, Roll: %f %f %f\n\n\r", yaw, pitch, roll);
				counter_wysw = 0;

				result = AS_readEncoder(&angleValue);
				switch(result){
				case 0:
					printf("Odczyt nieudany!\n");
					break;
				case 1:
					printf("%f\n", angleValue);
					break;
				case 2:
					printf("Magnes za daleko!\n");
					break;
				case 3:
					printf("Magnes za blisko!\n");
					break;
				}
			}

			if(I2CResult == 1){			//kontrola bledow I2C
				printf("Wystapily bledy timeout I2C!\n");
				I2CErrorCount += 1;
				I2CResult = 0;
			}
			else{
				I2CErrorCount = 0;
			}
			if(I2CErrorCount >= 10){
				I2CErrorCount = 0;
				resetI2C();
				printf("Wykonano reset I2C!\n\n\n\n\n\n\n\n\n");
			}

		}
	}
}

