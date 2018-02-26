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

void tim2_ititialize(){
	TIM_TimeBaseInitTypeDef tim;
	TIM_TimeBaseStructInit(&tim);
	tim.TIM_CounterMode = TIM_CounterMode_Up;
	tim.TIM_Prescaler = 6400 - 1;
	//tim.TIM_Period = 65000;
	TIM_TimeBaseInit(TIM2, &tim);
	TIM_Cmd(TIM2, ENABLE);
}

int main(void)
{

	I2C_InitTypeDef i2c;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	gpio_initialize();
	usart_initialize();
	nvic_initialize();
	tim2_ititialize();

	I2C_StructInit(&i2c);
	i2c.I2C_Mode = I2C_Mode_I2C;
	i2c.I2C_ClockSpeed = 100000;
	I2C_Init(I2C1, &i2c);
	I2C_Cmd(I2C1, ENABLE);

	SysTick_Config(SystemCoreClock / 1000);

	//MPU9250

	//int16_t gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
	//int16_t avg_temp_gyro[3], avg_temp_accel[3];
	int16_t temp[3];
	//int16_t rest = 0;
	uint8_t FS = 0;
	//uint8_t selfTest[6];
	//float factoryTrim[6] = {0.0};
	//float ResultST[6] = {0.0};

	float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0}, magbias[3] = {0, 0, 0};

	int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
	int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
	int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
	float ax, ay, az, gx, gy, gz, mx, my, mz;
	int t = 0;
	//float hardiron_correction[3] = {0.0};
	//float softiron_correction[3] = {0.0};

	// =======================TESTY KOMUNIKACJI===========================

	printf("Wyszukiwanie akcelerometru...\n");

	uint8_t who_am_i = mpu_read_reg(MPU9250_ADDR, WHO_AM_I);
	if (who_am_i == 0x71) {
		printf("Znaleziono akcelerometr MPU9250!\n");
	}
	else {
		printf("Niepoprawna odpowiedz ukladu(0x%02X)\n", who_am_i);
	}
	/*
	printf("Wyszukiwanie magnetometru...\n");

	who_am_i = 0;
	who_am_i = mpu_read_reg(AK8963_ADDRESS, AK8963_WHO_AM_I);
	if (who_am_i == 0x48) {
		printf("Znaleziono magnetometr AK8963!\n");
	}
	else {
		printf("Niepoprawna odpowiedz ukladu(0x%02X)\n", who_am_i);
	}

	 */
	printf("Reset MPU9250 \n");
	mpu_reset();

	mpu_calibrate(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
	printf("x gyro bias = %f\n\r", gyroBias[0]);
	printf("y gyro bias = %f\n\r", gyroBias[1]);
	printf("z gyro bias = %f\n\r", gyroBias[2]);
	printf("x accel bias = %f\n\r", accelBias[0]);
	printf("y accel bias = %f\n\r", accelBias[1]);
	printf("z accel bias = %f\n\r", accelBias[2]);
	delay_ms(2000);
	mpu_initialize();

	//========================================================================


	mpu_write_reg(MPU9250_ADDR, SMPLRT_DIV, 0x00); // Set gyro sample rate to 1 kHz
	mpu_write_reg(MPU9250_ADDR, CONFIG, 0x02); // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
	mpu_write_reg(MPU9250_ADDR, GYRO_CONFIG, 1<FS); // Set full scale range for the gyro to 250 dps
	mpu_write_reg(MPU9250_ADDR, ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
	mpu_write_reg(MPU9250_ADDR, ACCEL_CONFIG, 1<FS); // Set full scale range for the accelerometer to 2

	delay_ms(50);

	getAres(); // Get accelerometer sensitivity
	getGres(); // Get gyro sensitivity
	getMres(); // Get magnetometer sensitivity
	initAK8963(magCalibration);
	printf("%f %f %f", magCalibration[0], magCalibration[1], magCalibration[2]);
	printf("AK8963 initialized for active data mode....\n\r"); // Initialize device for active mode read of magnetometer
	printf("Accelerometer full-scale range = %f  g\n\r", 2.0f*(float)(1<<Ascale));
	printf("Gyroscope full-scale range = %f  deg/s\n\r", 250.0f*(float)(1<<Gscale));
	if(Mscale == 0) printf("Magnetometer resolution = 14  bits\n\r");
	if(Mscale == 1) printf("Magnetometer resolution = 16  bits\n\r");
	if(Mmode == 2) printf("Magnetometer ODR = 8 Hz\n\r");
	if(Mmode == 6) printf("Magnetometer ODR = 100 Hz\n\r");

	//magcalMPU9250(hardiron_correction, softiron_correction);

	mpu_ReadAccelData(temp);
	printf("Accel: %i %i %i\n", temp[0], temp[1], temp[2]);
	delay_ms(50);
	mpu_ReadGyroData(temp);
	printf("Gyro: %i %i %i \n", temp[0], temp[1], temp[2]);
	delay_ms(50);
	mpu_ReadMagData(temp);
	printf("Mag: %i %i %i\n", temp[0], temp[1], temp[2]);

	int counter_wysw = 0;


	/*printf("Accelerometer sensitivity is %f LSB/g \n\r", 1.0f/aRes);
			printf("Gyroscope sensitivity is %f LSB/deg/s \n\r", 1.0f/gRes);
			printf("Magnetometer sensitivity is %f LSB/G \n\r", 1.0f/mRes);*/
	magbias[0] = -73.48;  // User environmental x-axis correction in milliGauss, should be automatically calculated
	magbias[1] = 526.92;  // User environmental x-axis correction in milliGauss
	magbias[2] = -425.04;  // User environmental x-axis correction in milliGauss

	//magbias[0] = hardiron_correction[0];
	//magbias[1] = hardiron_correction[1];
	//magbias[2] = hardiron_correction[2];

	while(1) {



		// If intPin goes high, all data registers have new data
		if(mpu_read_reg(MPU9250_ADDR, INT_STATUS) & 0x01) {  // On interrupt, check if data ready interrupt

			mpu_ReadAccelData(accelCount);  // Read the x/y/z adc values
			// Now we'll calculate the accleration value into actual g's
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

			//printf("Mag: %i %i %i\n", magCount[0], magCount[1], magCount[2]);

			mx = (float)magCount[0]*mRes*magCalibration[0] - magbias[0];  // get actual magnetometer value, this depends on scale being set
			my = (float)magCount[1]*mRes*magCalibration[1] - magbias[1];
			mz = (float)magCount[2]*mRes*magCalibration[2] - magbias[2];



			//counter_wysw++;
			/*if (counter_wysw >= 60){
				printf("Accel: %f %f %f   ", ax, ay, az);
				printf("Gyro: %f %f %f   ", gx, gy, gz);
				printf("Mag: %f %f %f\n", mx, my, mz);
				counter_wysw = 0;
			}*/



			TIM_Cmd(TIM2, DISABLE);
			t = TIM_GetCounter(TIM2);
			deltat = (float)t/10000.0;


			MadgwickQuaternionUpdate(-ay, -ax, az, gy*PI/180.0f, gx*PI/180.0f, -gz*PI/180.0f,  mx,  my, mz);

			TIM_SetCounter(TIM2, 0);
			TIM_Cmd(TIM2, ENABLE);

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
			}

		}
	}
}

//}

