#include "frame.h"

// PRIVATE DEFINES
#define ESCAPE_CHAR 0x05
#define START_CHAR 0x0A
#define END_CHAR 0x0B
#define MIN_LENGTH 4  // escaped start and end characters

#define MCU_PIC

// PRIVATE FUNCTION PROTOTYPES
static bool byte_available(void);
static uint8_t get_byte(void);
static void send_byte_with_escape(uint8_t byte);
static void send_byte(uint8_t byte);


// PUBLIC FUNCTIONS

void Frame_Init(Frame_t* frame, uint8_t* buf, uint8_t len)
{
    frame->buf = buf;
    frame->len = len; 
    frame->i = 0;
    frame->escaped = false;
}

FrameResult_t Frame_Update(Frame_t* frame, uint8_t b)
{
    if(frame->escaped)
    {
        if(b == START_CHAR)
        {
            frame->i = 0;
        }
        else if(b == ESCAPE_CHAR)
        {
            if(frame->i >= frame->len)
            {
                return FRAME_OVERRUN;
            }
            frame->buf[frame->i] = ESCAPE_CHAR;
            frame->i++;
        }
        else if(b == END_CHAR)
        {
            return FRAME_COMPLETE;
        }
        else
        {
            return FRAME_INVALID_ESCAPE;
        }
    }
    else
    {
        if(b == ESCAPE_CHAR)
        {
            frame->escaped = true;
        }
        else
        {
            if(frame->i >= frame->len)
            {
                return FRAME_OVERRUN;
            }
            frame->buf[frame->i] = b;
            frame->i++;
        }
    }
    return FRAME_INCOMPLETE; 
}