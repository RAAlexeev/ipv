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
#include "EEPROM.hpp"
#include <functional>

class SignalChenal{
	static const uint32_t  BUFLEN  __attribute__((section(".ccmram")))= 4096u;
	static  SignalChenal  instances[];
	const Filter filter = Filter();
	std::function<uint16_t ()>const K;
	std::function<uint16_t ()>const range;
	CircularBuffer<float32_t,8> velocity_ =  CircularBuffer<float32_t,8>();
	CircularBuffer<float32_t,8> acceleration_ =  CircularBuffer<float32_t,8>();
	CircularBuffer<uint16_t,8> state = CircularBuffer<uint16_t,8>();
	float32_t y = 0;

	float32_t * buffer = buffer2;


public:
	float32_t buffer1[BUFLEN], buffer2[BUFLEN];
	SignalChenal(std::function<uint16_t(void)>const K, std::function<uint16_t (void)>const range):K(K),range(range){

	}
	static void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
	static void HAL_ADC_M1ConvCpltCallback(DMA_HandleTypeDef * 	hdma);
	void * swBuffer();
	void calc();
	float32_t getState(float32_t v)const{
		uint16_t s = state.average();
		if(s < 4096/2-1000)
			return NAN;
		if(s > 4096/2+1000)
			return INFINITY;
		return v;
	}
	 float32_t getVelocity()const {

		float32_t v = velocity_.average() * (range()/10.f);

		return getState(v+v*K()/1000);
	}
	 float32_t getAcceleration()const {
		float32_t a = acceleration_.average() * (range()/10.f);
		return getState(a+a*K()/1000);
	}
	static  SignalChenal*  getInstance(void* hadc){
		extern ADC_HandleTypeDef hadc1;
		return &instances[(hadc == &hadc1 || hadc == ADC1)?0:1];
	}
	void putState(uint16_t v){
		state.put(v);
	}


	static	void init();
};



#endif /* INC_INTEGRATOR_HPP_ */
