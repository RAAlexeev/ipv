/*
 * myUtils.hpp
 *
 *  Created on: 27 но€б. 2019 г.
 *      Author: alekseev
 */

#ifndef SRC_MYUTILS_HPP_
#define SRC_MYUTILS_HPP_

namespace myUtils{
inline void ITM_SendStr( char *s ){
	while(*s){
		ITM_SendChar(*s++ );
	}
}

}
#endif /* SRC_MYUTILS_HPP_ */
