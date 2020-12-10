/*
 * integrator.hpp
 *
 *  Created on: 3 дек. 2019 г.
 *      Author: alekseev
 */

#ifndef INC_INTEGRATOR_HPP_
#define INC_INTEGRATOR_HPP_

#include "arm_math.h"
#include "buffer.hpp"
#include "Filter.h"

class SignalChenal{
	static const uint32_t  BUFLEN  __attribute__((section(".ccmram")))= 4096u;
	static  SignalChenal  instances[];
	const Filter filter = Filter();
	CircularBuffer<float32_t,7> velocity_ =  CircularBuffer<float32_t,7>();
	CircularBuffer<float32_t,7> acceleration_ =  CircularBuffer<float32_t,7>();
	float32_t y = 0;

	float32_t * buffer = buffer2;


public:
	float32_t buffer1[BUFLEN], buffer2[BUFLEN];
	SignalChenal();
	static void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
	static void HAL_ADC_M1ConvCpltCallback(DMA_HandleTypeDef * 	hdma);
	void * swBuffer();
	void calc();
	inline float32_t getVelocity()const {

		float32_t v = velocity_.average();
		if(v < 0.5)return 0;
		return v;
	}
	inline float32_t getAcceleration()const {
		float32_t a = acceleration_.average();
		if(a < 0.155)return 0;
		return a;
	}
	static  SignalChenal*  getInstance(void* hadc){
		extern ADC_HandleTypeDef hadc1;
		return &instances[(hadc == &hadc1 || hadc == ADC1)?0:1];
	}

	static	void init();
};



#endif /* INC_INTEGRATOR_HPP_ */
