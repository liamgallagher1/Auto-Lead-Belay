#pragma once

#include <memory>
#include <mutex>

/**
* Important Usage Note: This library reserves one spare entry for queue-full detection
* Otherwise, corner cases and detecting difference between full/empty is hard.
* You are not seeing an accidental off-by-one.
*/

template <class T>
class CircularBuffer {
public:
  CircularBuffer(size_t size) :
    buf_(std::unique_ptr<T[]>(new T[size])),
    size_(size)
  {
  }

	
  void push(T item)
  {
  //  std::lock_guard<std::mutex> lock(mutex_);
    buf_[head_] = item;
    head_ = (head_ + 1) % size_;
  
    if(head_ == tail_) {
      tail_ = (tail_ + 1) % size_;
    }
  }


	T get(void)
  {
  //  std::lock_guard<std::mutex> lock(mutex_);

   if (empty()) {
     return T();
   }

   // Read data and advance the tail (we now have a free space)
   T val = buf_[tail_];
   tail_ = (tail_ + 1) % size_;
   return val;
  }


  T at(size_t indx);

	void reset(void);

	bool empty(void);

	bool full(void);

	size_t size(void);

private:
	std::mutex mutex_;
	std::unique_ptr<T[]> buf_;
	size_t head_ = 0;
	size_t tail_ = 0;
	size_t size_;
};
