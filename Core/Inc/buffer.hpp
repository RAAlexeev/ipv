/*
 * buffer.hpp
 *
 *  Created on: 27 но€б. 2019 г.
 *      Author: alekseev
 */

#ifndef SRC_BUFFER_HPP_
#define SRC_BUFFER_HPP_
#include "arm_math.h"

template <class T, size_t _size>
 class  CircularBuffer {
public:
	explicit CircularBuffer(){
		// empty
	}

	void put(T item){
	//	std::lock_guard<std::mutex> lock(mutex_);
;

		if(full_)
		{
			sum_ -= buf_[head_];
			tail_ = (tail_ + 1) % max_size_;
		}
		buf_[head_] = item;
		sum_ += buf_[head_];
		head_ = (head_ + 1) % max_size_;

		full_ = head_ == tail_;
	};
	//virtual T get();
	void reset(){
		//std::lock_guard<std::mutex> lock(mutex_);
		head_ = tail_;
		sum_= 0;
		full_ = false;
	};
	bool empty() const{
		//if head and tail are equal, we are empty
		return (!full_ && (head_ == tail_));
	};
	bool full() const{
		//If tail is ahead the head by 1, we are full
		return full_;
	};
	size_t capacity() const{
		return max_size_;
	};
	size_t size() const{
		size_t size = max_size_;

		if(!full_)
		{
			if(head_ >= tail_)
			{
				size = head_ - tail_;
			}
			else
			{
				size = max_size_ + head_ - tail_;
			}
		}

		return size;
	};
	T average(void)const{
		return sum_/size();
	};
private:
	T  buf_[_size];
	size_t head_ = 0;
	size_t tail_ = 0;
	const size_t max_size_ =_size;
	bool full_ = 0;
	float32_t sum_ = 0;
};


#endif /* SRC_BUFFER_HPP_ */
