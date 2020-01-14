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
#define LEN  4096u
class SignalChenal{
	static  SignalChenal instances[];

	CircularBuffer<float32_t,7> velocity_ =  CircularBuffer<float32_t,7>();
	float32_t y = 0;
	int16_t buffer1[LEN],buffer2[LEN];
	int16_t * buffer=buffer1;
	float32_t velocity;
public:
	SignalChenal();
	static void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
	void calc();
	inline float32_t getVelocity(){
		return velocity_.average();
	}
	static SignalChenal* getInstance(ADC_HandleTypeDef* hadc){
		return &instances[hadc->Instance == ADC1 ?0:1];
	}
	static	void init();
};



#endif /* INC_INTEGRATOR_HPP_ */
