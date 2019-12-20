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
class Recirculate{
	CircularBuffer<float32_t,7> velocity_ =  CircularBuffer<float32_t,7>();
	float32_t y = 0;
public:
	void calc(int16_t  * srcInt, float32_t * velocity);
};



#endif /* INC_INTEGRATOR_HPP_ */
