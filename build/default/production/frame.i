# 1 "frame.c"
# 1 "<built-in>" 1
# 1 "<built-in>" 3
# 288 "<built-in>" 3
# 1 "<command line>" 1
# 1 "<built-in>" 2
# 1 "C:/Users/leoja/.mchp_packs/Microchip/PIC18F-K_DFP/1.4.87/xc8\\pic\\include\\language_support.h" 1 3
# 2 "<built-in>" 2
# 1 "frame.c" 2
# 1 "./frame.h" 1
# 11 "./frame.h"
# 1 "C:\\Program Files\\Microchip\\xc8\\v2.41\\pic\\include\\c99\\stdint.h" 1 3



# 1 "C:\\Program Files\\Microchip\\xc8\\v2.41\\pic\\include\\c99\\musl_xc8.h" 1 3
# 5 "C:\\Program Files\\Microchip\\xc8\\v2.41\\pic\\include\\c99\\stdint.h" 2 3
# 22 "C:\\Program Files\\Microchip\\xc8\\v2.41\\pic\\include\\c99\\stdint.h" 3
# 1 "C:\\Program Files\\Microchip\\xc8\\v2.41\\pic\\include\\c99\\bits/alltypes.h" 1 3
# 127 "C:\\Program Files\\Microchip\\xc8\\v2.41\\pic\\include\\c99\\bits/alltypes.h" 3
typedef unsigned long uintptr_t;
# 142 "C:\\Program Files\\Microchip\\xc8\\v2.41\\pic\\include\\c99\\bits/alltypes.h" 3
typedef long intptr_t;
# 158 "C:\\Program Files\\Microchip\\xc8\\v2.41\\pic\\include\\c99\\bits/alltypes.h" 3
typedef signed char int8_t;




typedef short int16_t;




typedef __int24 int24_t;




typedef long int32_t;





typedef long long int64_t;
# 188 "C:\\Program Files\\Microchip\\xc8\\v2.41\\pic\\include\\c99\\bits/alltypes.h" 3
typedef long long intmax_t;





typedef unsigned char uint8_t;




typedef unsigned short uint16_t;




typedef __uint24 uint24_t;




typedef unsigned long uint32_t;





typedef unsigned long long uint64_t;
# 229 "C:\\Program Files\\Microchip\\xc8\\v2.41\\pic\\include\\c99\\bits/alltypes.h" 3
typedef unsigned long long uintmax_t;
# 23 "C:\\Program Files\\Microchip\\xc8\\v2.41\\pic\\include\\c99\\stdint.h" 2 3

typedef int8_t int_fast8_t;

typedef int64_t int_fast64_t;


typedef int8_t int_least8_t;
typedef int16_t int_least16_t;

typedef int24_t int_least24_t;
typedef int24_t int_fast24_t;

typedef int32_t int_least32_t;

typedef int64_t int_least64_t;


typedef uint8_t uint_fast8_t;

typedef uint64_t uint_fast64_t;


typedef uint8_t uint_least8_t;
typedef uint16_t uint_least16_t;

typedef uint24_t uint_least24_t;
typedef uint24_t uint_fast24_t;

typedef uint32_t uint_least32_t;

typedef uint64_t uint_least64_t;
# 144 "C:\\Program Files\\Microchip\\xc8\\v2.41\\pic\\include\\c99\\stdint.h" 3
# 1 "C:\\Program Files\\Microchip\\xc8\\v2.41\\pic\\include\\c99\\bits/stdint.h" 1 3
typedef int16_t int_fast16_t;
typedef int32_t int_fast32_t;
typedef uint16_t uint_fast16_t;
typedef uint32_t uint_fast32_t;
# 145 "C:\\Program Files\\Microchip\\xc8\\v2.41\\pic\\include\\c99\\stdint.h" 2 3
# 11 "./frame.h" 2

# 1 "C:\\Program Files\\Microchip\\xc8\\v2.41\\pic\\include\\c99\\stdbool.h" 1 3
# 12 "./frame.h" 2


typedef struct
{
    uint8_t* buf;
    uint8_t len;
    uint8_t i;
    _Bool escaped;
} Frame_t;

typedef enum
{
    FRAME_COMPLETE,
    FRAME_INCOMPLETE,
    FRAME_OVERRUN,
    FRAME_INVALID_ESCAPE
} FrameResult_t;



void Frame_Init(Frame_t* frame, uint8_t* buf, uint8_t len);
FrameResult_t Frame_Update(Frame_t* frame, uint8_t b);
# 1 "frame.c" 2
# 12 "frame.c"
static _Bool byte_available(void);
static uint8_t get_byte(void);
static void send_byte_with_escape(uint8_t byte);
static void send_byte(uint8_t byte);




void Frame_Init(Frame_t* frame, uint8_t* buf, uint8_t len)
{
    frame->buf = buf;
    frame->len = len;
    frame->i = 0;
    frame->escaped = 0;
}

FrameResult_t Frame_Update(Frame_t* frame, uint8_t b)
{
    if(frame->escaped)
    {
        if(b == 0x0A)
        {
            frame->i = 0;
        }
        else if(b == 0x05)
        {
            if(frame->i >= frame->len)
            {
                return FRAME_OVERRUN;
            }
            frame->buf[frame->i] = 0x05;
            frame->i++;
        }
        else if(b == 0x0B)
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
        if(b == 0x05)
        {
            frame->escaped = 1;
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
