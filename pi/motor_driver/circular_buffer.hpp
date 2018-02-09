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
  CircularBuffer(size_t size);
	
  void put(T item);

	T get(void);

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

