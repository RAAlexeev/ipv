/*
 * average.h
 *
 *  Created on: 25 но€б. 2019 г.
 *      Author: alekseev
 */

#ifndef SRC_AVERAGE_H_
#define SRC_AVERAGE_H_


template  <typename T>
class AverageCircle{
public:
	AverageCircle(uint8_t len){
		this->len = len;
		buf = new T[len];
		curLen=0;
		isFill=false;
	};
	void newValue(T val){
			if(this->curLen == this->len){
				this->curLen = 0;
				isFill = true;
			}
			this->buf[this->curLen] = val;
		};
	T get(void){
			float sum = 0;
			for(uint8_t i = 0; i < this->isFill?this->len:this->curLen; i++){
				sum += this->buf[i];
			}
			return reinterpret_cast<T>(sum/this->len);
		}
private:
		uint8_t len, curLen;
		bool isFill;
		T *buf;
};

#endif /* SRC_AVERAGE_H_ */
