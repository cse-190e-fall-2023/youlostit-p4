/*
 * i2c.c
 *
 *  Created on: Nov 14, 2023
 *      Author: Kenneth Tran, Do Heon Kim
 */

#include <stm32l475xx.h>


void i2c_init()
{
	// Enable clock for I2C2 peripheral and GPIOB
	RCC->APB1ENR1 |= RCC_APB1ENR1_I2C2EN; // Enable I2C2 clock
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN; // Enable GPIOB clock
	RCC->CR |= RCC_CR_HSION; // Enable msi16 clock

	// Configure PB10 and PB11 for alternate function open-drain (I2C)
	GPIOB->MODER &= ~(GPIO_MODER_MODER10 | GPIO_MODER_MODER11); // Clear mode bits for PB10 and PB11
    GPIOB->MODER |= GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1; // Set alternate function mode for PB10 and PB11

    GPIOB->OTYPER |= GPIO_OTYPER_OT_10 | GPIO_OTYPER_OT_11; // Set PB10 and PB11 as open-drain

    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD10 | GPIO_PUPDR_PUPD11); // Clear pull-up/pull-down bits
    GPIOB->PUPDR |= GPIO_PUPDR_PUPDR10_0 | GPIO_PUPDR_PUPDR11_0; // Set PB10 and PB11 with pull-up


    GPIOB->AFR[1] &= ~GPIO_AFRH_AFSEL10; // Clear bits for PB10 and PB11
    GPIOB->AFR[1] |= GPIO_AFRH_AFSEL10_2 | GPIO_AFRH_AFSEL11_2; // set PB10 and PB11 to AF4

    I2C2->CR1 &= ~I2C_CR1_PE; // Disable I2C2 peripheral to configure it
    RCC->CCIPR |= RCC_CCIPR_I2C2SEL_1; // set to HSI which is 16 MHz at restart

    // optional noise filter

    // setting up I2C2 timing register for f = 4 MHz --> 100 kHz baud rate
    I2C2->TIMINGR |= (0x3 << I2C_TIMINGR_PRESC_Pos) | (0x4 << I2C_TIMINGR_SCLDEL_Pos) | (0x2 << I2C_TIMINGR_SDADEL_Pos)
    				| (0xF << I2C_TIMINGR_SCLH_Pos) | (0x13 << I2C_TIMINGR_SCLL_Pos);

    I2C2->CR1 |= I2C_CR1_PE;

    // Set I2C mode to Master
//    I2C2->CR2 &= ~I2C_CR2_RD_WRN; // Write to slave
//    I2C2->CR2 &= ~I2C_CR2_ADD10; // 7-bit addressing mode
//    I2C2->CR2 |= I2C_CR2_HEAD10R; // 7 bit addressing standard; address --> dir bit
}

// for sending less than 255 bytes
uint8_t i2c_transaction(uint8_t address, uint8_t dir, uint8_t* data, uint8_t len)
{

    // following page 1298


    	// set read/write, addressing, secondary slave address, nbytes, reload=0, auto end=1, start
//    I2C2->CR2 |= (0x00 << I2C_CR2_RD_WRN_Pos) | (0x00 << I2C_CR2_ADD10_Pos) | (address << 0x01)|
//    		(len << I2C_CR2_NBYTES_Pos) | (I2C_CR2_START) | (I2C_CR2_AUTOEND);
    if (dir == 0){
    	// write 1 byte register address, 1 byte data
    	I2C2->CR2 &= ~(I2C_CR2_SADD | I2C_CR2_NBYTES | I2C_CR2_START | I2C_CR2_RD_WRN);
        I2C2->CR2 |= (0x00 << I2C_CR2_RD_WRN_Pos) | (0x00 << I2C_CR2_ADD10_Pos) | (address << 0x01)|
        		(len << I2C_CR2_NBYTES_Pos) | (I2C_CR2_START) | (I2C_CR2_AUTOEND);
    	for (uint8_t i = 0; i < len; i++) {
        	while ((I2C2->ISR & I2C_ISR_TXIS) != I2C_ISR_TXIS);
    		I2C2->TXDR = data[i];
    		}
    } else if (dir == 1){

    	// write register address
//    	while ((I2C2->ISR & I2C_ISR_TXIS) != I2C_ISR_TXIS);
//    	I2C2->TXDR = data[0];

    	// did not follow Texas instruments guide; also need to reset bits before setting in each read/write

    	I2C2->CR2 &= ~(I2C_CR2_SADD | I2C_CR2_NBYTES | I2C_CR2_START | I2C_CR2_RD_WRN);
    	I2C2->CR2 |= (0x01 << I2C_CR2_RD_WRN_Pos) | (0x00 << I2C_CR2_ADD10_Pos) | (address << 0x01)|
    	    		(len << I2C_CR2_NBYTES_Pos) | (I2C_CR2_START) | (I2C_CR2_AUTOEND);

    	// read 2 bytes?
    	for (uint8_t i = 0; i < len; i++){
    		while ((I2C2->ISR & I2C_ISR_RXNE) != I2C_ISR_RXNE);
    		data[i] = I2C2 -> RXDR;
    	}
    }
}









