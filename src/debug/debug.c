
#include <stdio.h>
#include "stm32f4xx_ll_usart.h"
#include "config.h"

extern uint8_t CmdSendBuf[USART3_SEND_BUF_SIZE];
extern uint8_t *CmdSendHead;
extern uint8_t * volatile CmdSendTail;

struct __FILE
{
  int handle;
  /* Whatever you require here. If the only file you are using is */
  /* standard output using printf() for debugging, no file handling */
  /* is required. */
};

/* FILE is typedefed in stdio.h. */

FILE __stdout;

void sendchar(uint8_t ch)
{
    uint8_t *next; /**< point to the next free space */
    
    *CmdSendHead = ch;
    if(CmdSendHead == CmdSendBuf + CMD_SEND_BUF_SIZE - 1)
    {
        next = CmdSendBuf;
    }
    else
    {
        next = CmdSendHead + 1;
    }
    while(next == CmdSendTail)  // CmdSendTail must be declared as volatile
    {
        // buffer full, wait for at least one byte space
    }
    CmdSendHead = next;
    LL_USART_EnableIT_TXE(USART3); // start transmition loop by triggering the serial interrupt
}

int fputc(int ch, FILE *f)
{
    if(ch == '\n')
    {
        sendchar('\r'); // insert \r
    }
    sendchar(ch);
    
    return ch;
}

//int ferror(FILE *f)
//{
//  /* Your implementation of ferror(). */
//  return 0;
//}

/**
 * Trace an array output for debuging.
 *
 * \param[in] array The array name or address to trace output.
 * \param[in] len The number of bytes to trace output.
 *
 * \note The array size must be no more than 256. If the size is 256 set len to 0.
 */
void TraceArray(uint8_t array[], uint32_t len)
{
    uint32_t i = 0;
    
    while(1)
    {
        TRACE1("%02X", array[i]);
        i++;
        if(i != len)
        {
            TRACE(" ");
        }
        else
        {
            TRACE("\n");
            break;
        }
    }
}
