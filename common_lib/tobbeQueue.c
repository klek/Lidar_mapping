/*
 * queue.c
 *
 *  Created on: Apr 27, 2015
 *      Author: orientera
 */

#include <stdio.h>
#include <stdint.h>
#include "tobbeQueue.h"

// Contructor
Queue _Queue () {
	Queue s;

	s.front = -1;
	s.rear = -1;

	return s;
}

// Enqueue is performed by inserting at the end of the array
void enqueue(Queue* q1, Queue_node d1) {
	// Allocate memory for the incomming data using malloc
	//Queue_node* data = (Queue_node*)malloc(sizeof(*d1));

	// Test if the queue is full
	if ( isFull(q1) ) {
		// Well if the queue is full then we have a problem
		// For now, lets enter an infinite loop
		//while (1) {

		//}
	}
	// If the Queue is empty we increment the front and the rear
	else if (isEmpty(q1)) {
		q1->front = q1->rear = 0;
	}
	// The queue was not empty, now we increment rear
	else {
		q1->rear = (q1->rear + 1) % MAX_NR_EL_QUEUE;
	}

	// Point the rear slot to the data
	q1->que[q1->rear] = d1;

}

// Dequeue should simply deallocate the memory stored at the front slot and then increment the front
void dequeue(Queue* q1) {
	int i = 0;

	// First test if the queue is empty or not
	if (isEmpty(q1)) {
		// Queue is empty
		// Nothing to do..
	}
	// The queue contains only one element, when this is removed the queue should be marked as empty
	else if ( q1->front == q1->rear ){
		// Deallocate the data stored at slot front
		//free(q1->que[q1->front]);

		// For safety, repoint the memory-address to null
		//q1->que[q1->front] = NULL;

		// Maybe we should set the front slot to zero
		for (i = 0; i < MAX_MSG_SIZE; i++) {
			q1->que[q1->front].data[i] = 0;
		}

		// Set front and rear to negative value to indicate that the queue is empty
		q1->front = -1;
		q1->rear = -1;
	}
	else {
		// Deallocate the memory used
		//free(q1->que[q1->front]);

		// For safety, repoint the memory-address to null
		//q1->que[q1->front] = NULL;

		// Maybe we should set the front slot to zero
		for (i = 0; i < MAX_MSG_SIZE; i++) {
			q1->que[q1->front].data[i] = 0;
		}

		// Now we need to increment the front
		// Test if the front is at the end
		q1->front = (q1->front + 1) % MAX_NR_EL_QUEUE;
	}
}

// This function should simply return the item in the front of the queue
// In this case it will be a pointer to uint8_t pointer
Queue_node getFront(Queue* q1) {
	// Return the item in the front of the queue
	return (q1->que[q1->front]);
}

// Test to see if the queue is empty or not
int isEmpty(Queue* q1) {
	if ( q1->front == -1 && q1->rear == -1 ) {
		return 1;
	}
	else return 0;
}

// Test to see if the queue is full or not
int isFull(Queue* q1) {
	if( (q1->rear + 1 ) % MAX_NR_EL_QUEUE == q1->front ){
		return 1;
	}
	else
		return 0;
}
