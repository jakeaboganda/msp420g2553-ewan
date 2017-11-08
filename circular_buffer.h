/*
 * circular_buffer.h
 *
 *  Created on: 2017/10/25
 *      Author: jake
 */

#ifndef CIRCULAR_BUFFER_H_
#define CIRCULAR_BUFFER_H_

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#define CIRCULAR_BUFFER_SIZE 256

typedef volatile struct {
    uint8_t start;
    uint8_t end;
    uint8_t buffer[CIRCULAR_BUFFER_SIZE];
} circular_buffer_t;


#define CIRCULAR_BUFFER_INIT {0, 0}

bool circularBuffer_isEmpty(circular_buffer_t *cbuffer);
bool circularBuffer_pushByte(circular_buffer_t *cbuffer, uint8_t byte);
uint8_t circularBuffer_popByte(circular_buffer_t *cbuffer);


#endif /* CIRCULAR_BUFFER_H_ */
