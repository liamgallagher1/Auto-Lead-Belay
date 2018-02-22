#pragma once

#include <cstdio>

#include <memory>
#include <mutex>

// Mostly not authored by liam
// https://github.com/embeddedartistry/embedded-resources/blob/master/examples/cpp/circular_buffer.cpp

#define EMPTY_CONSTRUCTOR_SIZE 1000


/**
* Important Usage Note: This library reserves one spare entry for queue-full detection
* Otherwise, corner cases and detecting difference between full/empty is hard.
* You are not seeing an accidental off-by-one.
*/

template <class T>
class CircularBuffer {
public:
  
  // No argument constructor
  CircularBuffer() :
    buf_(std::unique_ptr<T[]>(new T[EMPTY_CONSTRUCTOR_SIZE])),
    size_(EMPTY_CONSTRUCTOR_SIZE)
  {
  }

  CircularBuffer(size_t size) :
  	buf_(std::unique_ptr<T[]>(new T[size])),
  	size_(size)
  {
  }
  
  // TODO maybe have get pointer method

  void push_front(T item)
  {
  	std::lock_guard<std::mutex> lock(mutex_);

  	buf_[head_] = item;
  	head_ = (head_ + 1) % size_;

  	if(head_ == tail_)
  	{
  		tail_ = (tail_ + 1) % size_;
  	}
    add_count_++;
  }

  T pop_back(void)
  {
  	std::lock_guard<std::mutex> lock(mutex_);

  	if(empty())
  	{
  		return T();
  	}

  	//Read data and advance the tail (we now have a free space)
  	auto val = buf_[tail_];
  	tail_ = (tail_ + 1) % size_;

  	return val;
  }

  // Used to get the element, with the head being the 0th element
  // No safety check on validity of indx
  T operator[] (int indx) const
  {
    // TODO I only think this is right
    int buf_indx = (head_ - 1 + size_ - indx) % size_;
    return buf_[buf_indx];
  }

  void reset(void)
  {
  	std::lock_guard<std::mutex> lock(mutex_);
  	head_ = tail_;
    add_count_ = 0;
  }

  bool empty(void) const
  {
  	//if head and tail are equal, we are empty
  	return head_ == tail_;
  }

  bool full(void) const
  {
  	//If tail is ahead the head by 1, we are full
  	return ((head_ + 1) % size_) == tail_;
  }

  size_t space(void) const
  {
  	return size_ - 1;
  }

  // number of elements currently in it
  // TODO make real
  size_t num_elements(void) const 
  {
    if (full()) return space();
    if (empty()) return 0; 
    return head_ - 1;
  }

  long add_count(void) const
  {
    return add_count_;
  }

private:
  std::mutex mutex_;
  std::unique_ptr<T[]> buf_;
  size_t head_ = 0;
  size_t tail_ = 0;
  long add_count_ = 0;
  size_t size_;
};
