/*
 * queue
 *
 *  Created on: Apr 27, 2015
 *      Author: orientera
 */

#ifndef TOBBEQUEUE_H_
#define TOBBEQUEUE_H_

#define MAX_NR_EL_QUEUE		30
#define MAX_MSG_SIZE		4
#define LENGTH_TO_TRAV		2
#define MSG					1
#define LENGTH				0
#define READ_DIST			3

// The node in the Queue
typedef struct {
	uint16_t data[MAX_MSG_SIZE];
} Queue_node;


// The queue class
typedef struct {
	 Queue_node que[MAX_NR_EL_QUEUE];
	 int front, rear;
} Queue;

void enqueue(Queue* q1, Queue_node data);
void dequeue(Queue* q1);
Queue_node getFront(Queue* q1);
int isEmpty(Queue* q1);
int isFull(Queue* q1);



#endif /* TOBBEQUEUE_H_ */
