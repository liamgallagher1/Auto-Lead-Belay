/**
 * Circular Queue Implementation
 *
 * This queue will handle any type of objects only changing
 * the typedef above.
 *
 * @author Bruno Alano Medina
 * @version 1.0.0
 *
 * Modified by Liam Gallagher liam.gallagher0@gmail.com in January 2018
 */

/* Standard Libraries */
#include <stdbool.h>

#ifndef _QUEUE_H_INCLUDED
#define _QUEUE_H_INCLUDED

/**
 * The type of element that we are storing in the
 * queue
 */
typedef double Element;

/**
 * Queue Structure
 */
typedef struct _queue {
	/* Store the vector */
	Element *vector;

	/* The element in the head of queue */
	size_t head;

	/* The last element in queue */
	size_t tail;

	/* Size of queue */
	size_t size;
} Queue;

/**
 * Creates a Queue with Size
 * 
 * @param  size Length of vector of Element's
 * @return      return a Queue pointer (you shoud free it)
 */
Queue *createQueue(size_t size);

/**
 * Dealloc a Queue
 * 
 * @param q Queue object
 */
void deallocateQueue(Queue *q);

/**
 * Add Element into Queue
 * 
 * @param q Queue object
 * @param n Element to be add
 */
void enqueue(Queue *q, Element n);

/**
 * Retrieve Element from Queue
 * 
 * @param  q  Queue object
 * @return    Element (see typedef definition)
 */
Element dequeue(Queue *q);

/**
 * Is this queue full?
 * 
 * @param  q Queue object
 * @return   boolean (true if full)
 */
bool full(Queue *q);

/**
 * Is this queue empty?
 * 
 * @param  q Queue object
 * @return   boolean (true if empty)
 */
bool empty(Queue *q);

/** 
 * gets the element at this relative index 
 *
 * @param q Queue object
 * @return Element
 */ 
Element at(Queue *q, size_t indx); 

#endif
