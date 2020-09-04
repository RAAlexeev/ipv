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
	static const uint32_t  BUFLEN = 4096u;
	static  SignalChenal instances[];
	Filter filter = Filter();
	CircularBuffer<float32_t,7> velocity_ =  CircularBuffer<float32_t,7>();
	CircularBuffer<float32_t,7> acceleration_ =  CircularBuffer<float32_t,7>();
	float32_t y = 0;
	float32_t buffer1[BUFLEN],buffer2[BUFLEN];
	float32_t * buffer=buffer1;


public:

	SignalChenal();
	static void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
	void * swBuffer();
	void calc();
	inline float32_t getVelocity(){
		return velocity_.average();
	}
	inline float32_t getAccelerarion(){
		return acceleration_.average();
	}
	static SignalChenal* getInstance(ADC_HandleTypeDef* hadc){
		return &instances[hadc->Instance == ADC1 ?0:1];
	}
	static	void init();
};



#endif /* INC_INTEGRATOR_HPP_ */
