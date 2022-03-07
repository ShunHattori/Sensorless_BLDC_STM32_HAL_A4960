#pragma once
#include "main.h"

class A4960_Helper {
public:
	A4960_Helper(SPI_HandleTypeDef*, GPIO_TypeDef*, uint16_t,
			TIM_HandleTypeDef*, uint32_t);
	~A4960_Helper();

	void init();
	void run(int32_t pwm_signed);

private:
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef *GPIOx;
	uint16_t GPIO_Pin;
	TIM_HandleTypeDef *htim;
	uint32_t channel;

	int32_t pwm_prev;
	uint8_t direction_bit;

	int8_t sgn(int32_t pwm_signed);
	uint8_t update_direction(int sign);

	int16_t write_register(uint8_t *reg);
	int16_t read_register(uint8_t *reg);
	int16_t write_read_register(uint8_t *reg);

	const double MAX_TIMER_CCR_RATIO = 0.5;
	const uint8_t A4960_REG_WRITE = 0b00010000;
	const uint8_t A4960_REG_BLANK = 0b00000000;
	const uint8_t A4960_REG_VREF = 0b00100000;
	const uint8_t A4960_REG_PWM = 0b01000000;
	const uint8_t A4960_REG_HOLD = 0b01100000;
	const uint8_t A4960_REG_START = 0b10000000;
	const uint8_t A4960_REG_RAMP = 0b10100000;
	const uint8_t A4960_REG_MASK = 0b11000000;
	const uint8_t A4960_REG_RUN = 0b11100000;
	const uint8_t A4960_REG_DIAG = 0b11000000;  //Read only
};

A4960_Helper::A4960_Helper(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOx,
		uint16_t GPIO_Pin, TIM_HandleTypeDef *htim, uint32_t channel) {
	this->hspi = hspi;
	this->GPIOx = GPIOx;
	this->GPIO_Pin = GPIO_Pin;
	this->htim = htim;
	this->channel = channel;
}

A4960_Helper::~A4960_Helper() {
}

void A4960_Helper::init() {
	uint8_t spi_data[2];
	HAL_TIM_PWM_Start(this->htim, this->channel);
	__HAL_TIM_SET_COMPARE(this->htim, this->channel, 0);
	for (uint8_t i = 0; i < 5; i++) {
//		//setting Hold Time
//		spi_data[0] = A4960_REG_WRITE | A4960_REG_HOLD | 0b00000000;
//		spi_data[1] = 0b01010010; //HT[3:0]
//		HAL_GPIO_WritePin(GPIOx, GPIO_Pin, RESET);
//		HAL_SPI_Transmit(hspi, spi_data, (sizeof(spi_data) / sizeof(uint8_t)),
//				1000);
//		HAL_GPIO_WritePin(GPIOx, GPIO_Pin, SET);
//		__NOP();
//
//		//setting Start commutation time
//		spi_data[0] = A4960_REG_WRITE | A4960_REG_START | 0b00000000;
//		spi_data[1] = 0b11110010; //SC[3:0]
//		HAL_GPIO_WritePin(GPIOx, GPIO_Pin, RESET);
//		HAL_SPI_Transmit(hspi, spi_data, (sizeof(spi_data) / sizeof(uint8_t)),
//				1000);
//		HAL_GPIO_WritePin(GPIOx, GPIO_Pin, SET);
//		__NOP();
//
//		//setting Phase Advance
//		spi_data[0] = A4960_REG_WRITE | A4960_REG_RAMP | 0b11000000; //PA[3:0]
//		spi_data[1] = 0b10000000; //Default
//		HAL_GPIO_WritePin(GPIOx, GPIO_Pin, RESET);
//		HAL_SPI_Transmit(hspi, spi_data, (sizeof(spi_data) / sizeof(uint8_t)),
//				1000);
//		HAL_GPIO_WritePin(GPIOx, GPIO_Pin, SET);
//		__NOP();

		//setting Run State
		spi_data[0] = A4960_REG_WRITE | A4960_REG_RUN | 0b00000010; //Default
		spi_data[1] = 0b00001001; //RUN enable
		write_register(spi_data);
	}
}

void A4960_Helper::run(int32_t pwm_signed) {
	if (sgn(pwm_signed) != sgn(pwm_prev) && sgn(pwm_signed) != 0) {
		direction_bit = update_direction(sgn(pwm_signed));
	}

	int32_t timer_CCR = 0;
	if (sgn(pwm_signed) == -1) {
		timer_CCR = -pwm_signed;
	} else {
		timer_CCR = pwm_signed;
	}

	if (timer_CCR < 0) {
		return;
	} else if (timer_CCR >= this->htim->Init.Period * MAX_TIMER_CCR_RATIO) {
		timer_CCR = this->htim->Init.Period * MAX_TIMER_CCR_RATIO;
	}

	uint8_t spi_data[2];
	spi_data[0] = A4960_REG_WRITE | A4960_REG_RUN | 0b00000010; //Default
	spi_data[1] = 0b00001001 | direction_bit; //RUN enable
	write_register(spi_data);
	__HAL_TIM_SET_COMPARE(this->htim, this->channel, timer_CCR);
	pwm_prev = pwm_signed;
}

uint8_t A4960_Helper::update_direction(int sign) {
	//safety
	if (sign == 0) {
		return direction_bit;
	}

	uint8_t spi_data[2];
	uint8_t dir_bit = 0;
	if (sign == 1) {
		dir_bit = 0b00000000;
	} else if (sign == -1) {
		dir_bit = 0b00000010;
	} else {
		return direction_bit;
	}

	for (int i = 0; i < 3; i++) {
		//setting Run to disable and change direction
		spi_data[0] = A4960_REG_WRITE | A4960_REG_RUN | 0b00000010; //Default
		spi_data[1] = 0b00001000 | dir_bit; //RUN disable
		write_register(spi_data);
	}
	return dir_bit;
}

int16_t A4960_Helper::write_register(uint8_t *data) {
	HAL_GPIO_WritePin(this->GPIOx, this->GPIO_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(this->hspi, data, 2, 1000);
	HAL_GPIO_WritePin(this->GPIOx, this->GPIO_Pin, GPIO_PIN_SET);
	__NOP();
	return 0;
}

int8_t A4960_Helper::sgn(int32_t pwm_signed) {
	if (pwm_signed == 0)
		return 0;
	if (pwm_signed > 0)
		return 1;
	return -1;
}
