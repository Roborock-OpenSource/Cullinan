
#include "config.h"
#include <string.h>
#include "core_cm4.h"   // for __INLINE


void sendchar(uint8_t ch);
void CmdHandler(void);

uint8_t CmdRecvBuf[CMD_RECV_BUF_SIZE];
uint8_t *CmdRecvHead;
uint8_t *CmdRecvTail;
uint8_t CmdSendBuf[CMD_SEND_BUF_SIZE];
uint8_t *CmdSendHead;
uint8_t * volatile CmdSendTail;

uint8_t CmdLineBuf[CMD_LINE_SIZE];

uint32_t SendcharEnable;

void CmdInit(void)
{
    CmdRecvHead = CmdRecvBuf;
    CmdRecvTail = CmdRecvBuf;
    CmdSendHead = CmdSendBuf;
    CmdSendTail = CmdSendBuf;
    SendcharEnable = 1; // enable output
}

#define IS_CMD_RECV_EMPTY()     (CmdRecvTail == CmdRecvHead)
#define IS_CMD_RECV_NOT_EMPTY() (CmdRecvTail != CmdRecvHead)
#define GETC()                  (*CmdRecvTail)

__STATIC_INLINE uint8_t CmdDeque(void)
{
    uint8_t c;
    
    c = *CmdRecvTail; // read receive data
    if(CmdRecvTail == CmdRecvBuf + CMD_RECV_BUF_SIZE - 1)
    {
        CmdRecvTail = CmdRecvBuf;
    }
    else
    {
        CmdRecvTail++;
    }
    return c;
}


uint32_t CmdRecv(void)
{
    static uint8_t i;
    uint8_t c;
    uint32_t RetVal;
    
    while(IS_CMD_RECV_NOT_EMPTY())
    {
        SendcharEnable = 1; // enable output
        c = CmdDeque();
        // control character
        if(c == '\b')
        {// backspace
            sendchar(c); // echo
            sendchar(' '); // space
            sendchar(c);
            if(i != 0)
            {
                i--;
            }
            SendcharEnable = 0; // disable output
        }
        else if(c == '\r')
        {// CR
            sendchar('\r'); sendchar('\n'); // echo
            CmdLineBuf[i] = '\0'; // end of string
            RetVal = i;
            i = 0;
            return RetVal;    // return the string length
        }
        else if(c == '\n')
        {// LF
            sendchar('\r'); sendchar('\n'); // echo
            CmdLineBuf[i] = '\0'; // end of string
            RetVal = i;
            i = 0;
            return RetVal;    // return the string length
        }
        else
        {// receive command data
            if(i < CMD_LINE_SIZE - 1)
            {// command line buffer not full
//                if(c == '\\')
//                {// command header
//                    // wait for all data sent
//                    while(g_bSerialBusy) ;
//                }
                sendchar(c); // echo
                CmdLineBuf[i] = c;
                i++;
            }
            else
            {// command line buffer full
                // discard the character
            }
            SendcharEnable = 0; // disable output
        }
    }
    return 0;
}


void cmd(void)
{
    if(CmdRecv() != 0)
    {
        CmdHandler();
    }
}

