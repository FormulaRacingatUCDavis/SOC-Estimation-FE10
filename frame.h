/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

#ifndef FRAME_H
#define	FRAME_H

#include "stdint.h"
#include "stdbool.h"

typedef struct
{
    uint8_t* buf;
    uint8_t len; 
    uint8_t i; 
    bool escaped;
} Frame_t;

typedef enum
{
    FRAME_COMPLETE,
    FRAME_INCOMPLETE,
    FRAME_OVERRUN,
    FRAME_INVALID_ESCAPE
} FrameResult_t;


// PUBLIC FUNCTION PROTOTYPES
void Frame_Init(Frame_t* frame, uint8_t* buf, uint8_t len);
FrameResult_t Frame_Update(Frame_t* frame, uint8_t b);

#endif	/* XC_HEADER_TEMPLATE_H */

