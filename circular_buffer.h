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

#define circularBuffer_isEmpty(cbuffer) (((cbuffer)->start == (cbuffer)->end))
#define circularBuffer_popByte(cbuffer) ((cbuffer)->buffer[(cbuffer)->start++])
#define circularBuffer_pushByte(cbuffer, byte) \
{ \
    (cbuffer)->buffer[(cbuffer)->end] = byte; \
    (cbuffer)->end = (cbuffer)->end + 1; \
    if(circularBuffer_isEmpty((cbuffer))) \
    { \
        (cbuffer)->start++; \
    } \
}

#endif /* CIRCULAR_BUFFER_H_ */
