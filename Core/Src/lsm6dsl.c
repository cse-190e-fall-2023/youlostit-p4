/*
 * lsm6dsl.c
 *
 *  Created on: Nov 14, 2023
 *      Author: Kenneth Tran, Do Heon Kim
 */
#include <stm32l475xx.h>
#include "i2c.h"

// accelerometer connected to ground
uint8_t accelerometer_write = 0xD4;
uint8_t accelerometer_read = 0xD5;

// when we only care about accelerometer 7-bit address
uint8_t accelerometer = 0b1101010;
uint8_t CTRL1_XL = 0x10;
uint8_t INT1_CTRL = 0x0D;
uint8_t CTRL2_G = 0x11;

void lsm6dsl_init(){

	// data to be sent, [0] is register address, [1] is data
	//	CTRL1->XL = 0X60;
	//	INT1->CTRL = 0X01;
	//	CTRL1->XL = 0X60;
	//	INT1->CTRL = 0X02;

	i2c_init();

	// i2c_transaction(uint8_t address, uint8_t dir, uint8_t* data, uint8_t len)

	//EXTI setup
//	GPIOD->M
	// turn on accelerometer 1.6-12.5 Hz (low performance), data-ready interrupt
	uint8_t data[2] = {CTRL1_XL, 0xB0};
	i2c_transaction(accelerometer, 0x00, data, 0x02);

	uint8_t data2[2] = {INT1_CTRL, 0x01};
	i2c_transaction(accelerometer, 0x00, data2, 0x02);


}




void lsm6dsl_read_xyz(int16_t* x, int16_t* y, int16_t* z){

	uint8_t OUTX_L_XL = 0x28;
	uint8_t OUTX_H_XL = 0x29;

	uint8_t OUTY_L_XL = 0x2A;
	uint8_t OUTY_H_XL = 0x2B;

	uint8_t OUTZ_L_XL = 0x2C;
	uint8_t OUTZ_H_XL = 0x2D;

	uint8_t xL[1] = {OUTX_L_XL};
	uint8_t xH[1] = {OUTX_H_XL};
	uint8_t yL[1] = {OUTY_L_XL};
	uint8_t yH[1] = {OUTY_H_XL};
	uint8_t zL[1] = {OUTZ_L_XL};
	uint8_t zH[1] = {OUTZ_H_XL};

	// read accelerometer data
	i2c_transaction(accelerometer, 0x00, xL, 0x01);
	i2c_transaction(accelerometer, 0x01, xL, 0x01);

	i2c_transaction(accelerometer, 0x00, xH, 0x01);
	i2c_transaction(accelerometer, 0x01, xH, 0x01);

	i2c_transaction(accelerometer, 0x00, yL, 0x01);
	i2c_transaction(accelerometer, 0x01, yL, 0x01);

	i2c_transaction(accelerometer, 0x00, yH, 0x01);
	i2c_transaction(accelerometer, 0x01, yH, 0x01);

	i2c_transaction(accelerometer, 0x00, zL, 0x01);
	i2c_transaction(accelerometer, 0x01, zL, 0x01);

	i2c_transaction(accelerometer, 0x00, zH, 0x01);
	i2c_transaction(accelerometer, 0x01, zH, 0x01);

	// store values with type casting
	*x = (int16_t)(((uint16_t)xH[0] << 8) | xL[0]);
	*y = (int16_t)(((uint16_t)yH[0] << 8) | yL[0]);
	*z = (int16_t)(((uint16_t)zH[0] << 8) | zL[0]);

}



