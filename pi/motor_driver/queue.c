/**
 * Circular Queue Implementation
 *
 * This queue will handle any type of objects only changing
 * the typedef above.
 *
 * @author Bruno Alano Medina
 * @version 1.0.0
 */

/* Standard Libraries */
#include <stdlib.h>
#include <stdio.h>

/* Queue Library */
#include "queue.h"

Queue *createQueue(size_t size)
{
	Queue *q = calloc(sizeof(Queue), 1);
	q->vector = calloc(sizeof(Element) * size, 1);
	q->size = size;
	q->head = 0;
	q->tail = 0;
	return q;
}

void deallocateQueue(Queue *q)
{
	free(q->vector);
	free(q);
}

bool full(Queue *q)
{
	return (q->head == ((q->tail + 1) % q->size));
}

bool empty(Queue *q)
{
	return (q->tail == q->head);
}

void enqueue(Queue *q, Element n)
{
	if (full(q))
	{
		printf("enqueue on full queue\n");
		exit(0);
	}
	q->vector[q->tail] = n;
	q->tail = (q->tail + 1) % q->size;
}

Element dequeue(Queue *q)
{
	if (empty(q))
	{
		printf("dequeue on empty queue\n");
		exit(0);
	}

	Element temp = q->vector[q->head];
	q->head = (q->head + 1) % q->size;
	return temp;
}

Element at(Queue *q, size_t indx)
{
    if (empty(q)) {
        printf("at called on empty queue\n");
        return -1;
    } else if (indx >= q->size) {
        printf("at called when indx out ouf bounds\n");
        return -1;
    }
    Element here = q->vector[(q->head + indx) % q->size];  
    return here;
}

