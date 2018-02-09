#include <cstdio>

#include "circular_buffer.hpp"

/**
* Important Usage Note: This library reserves one spare entry for queue-full detection
* Otherwise, corner cases and detecting difference between full/empty is hard.
* You are not seeing an accidental off-by-one.
*/

template <typename T>
CircularBuffer<T>::CircularBuffer(size_t size) :
  buf_(std::unique_ptr<T[]>(new T[size])),
  size_(size)
{
}

template <typename T>
void CircularBuffer<T>::push(T item)
{
//  std::lock_guard<std::mutex> lock(mutex_);

  buf_[head_] = item;
  head_ = (head_ + 1) % size_;

  if(head_ == tail_) {
    tail_ = (tail_ + 1) % size_;
  }
}

template <typename T>
T CircularBuffer<T>::at(size_t indx)
{
//  std::lock_guard<std::mutex> lock(mutex_);

  return buf_[(head_ + 2 * size_ - 1 - indx) % size_];
}


template <typename T>
T CircularBuffer<T>::get(void)
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

template <typename T>
void CircularBuffer<T>::reset(void)
{
//  std::lock_guard<std::mutex> lock(mutex_);
  head_ = tail_;
}

template <typename T>
bool CircularBuffer<T>::empty(void)
{
	// if head and tail are equal, we are empty
  return head_ == tail_;
}

template <typename T>
bool CircularBuffer<T>::full(void)
{
	//If tail is ahead the head by 1, we are full
  return ((head_ + 1) % size_) == tail_;
}

template <typename T>
size_t CircularBuffer<T>::size(void)
{
  return size_ - 1;
}

