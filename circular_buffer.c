
#include "circular_buffer.h"


bool circularBuffer_isEmpty(circular_buffer_t *cbuffer)
{
    return (cbuffer->start == cbuffer->end);
}

bool circularBuffer_pushByte(circular_buffer_t *cbuffer, uint8_t byte)
{
    cbuffer->buffer[cbuffer->end] = byte;
    cbuffer->end = cbuffer->end + 1;
    if(cbuffer->start == cbuffer->end)
    {
        cbuffer->start++;
    }
    return true;
}

//assumes buffer is not empty
uint8_t circularBuffer_popByte(circular_buffer_t *cbuffer)
{
    return cbuffer->buffer[cbuffer->start++];
}

