
#include <stdio.h>
#include <stdint.h>
#include "stm32f4xx_ll_usart.h"
#include "config.h"
#include "LDS.h" 

#define IS_UART5_RECV_BUF_NOT_EMPTY() (UART5_RecvTail != UART5_RecvHead)

extern uint8_t UART5_SendBuf[UART5_SEND_BUF_SIZE];
extern uint8_t * volatile UART5_SendTail;
extern uint8_t *UART5_SendHead;
extern uint8_t UART5_RecvBuf[UART5_RECV_BUF_SIZE];
extern uint8_t *UART5_RecvTail;
extern uint8_t *UART5_RecvHead;

extern uint32_t SpeedCtrlFlag;

uint8_t lds_buf[LDS_BUF_SIZE];
uint8_t lds_buf_cnt;
struct _LDS_PID LDS_PID;

void UART5_TX_enque(uint8_t ch)
{
    uint8_t *next; /**< point to the next free space */
    
    *UART5_SendHead = ch;
    if (UART5_SendHead == UART5_SendBuf + UART5_SEND_BUF_SIZE - 1)
    {
        next = UART5_SendBuf;
    }
    else
    {
        next = UART5_SendHead + 1;
    }
    while (next == UART5_SendTail) // UART5_SendTail must be declared as volatile
    {
        // buffer full, wait for at least one byte space
    }
    UART5_SendHead = next;
    LL_USART_EnableIT_TXE(UART5); // start transmition loop by triggering the serial interrupt
}

__STATIC_INLINE uint8_t UART5_RX_deque(void)
{
    uint8_t c;
    
    c = *UART5_RecvTail; // read receive data
    if(UART5_RecvTail == UART5_RecvBuf + UART5_RECV_BUF_SIZE - 1)
    {
        UART5_RecvTail = &UART5_RecvBuf[0];
    }
    else
    {
        UART5_RecvTail++;
    }
    return c;
}

__INLINE void UART5_RX_SetEmpty(void)
{
    UART5_RecvTail = UART5_RecvHead;
}

int32_t lds_check_info_packet(void)
{
    uint16_t checksum = 0;
    uint32_t i;
    
    // skip the sync
    for (i = 1; i < 83; i += 2)
    {
        checksum += *(uint16_t *)&lds_buf[i];
    }
    
    if (checksum == *(uint16_t *)&lds_buf[83])
    {// checksum OK
        return 0;
    }
    else
    {// checksum FAIL
        TRACE("\ninfo packet checksum FAIL!\n");
        TRACE_ARRAY(lds_buf, 84 + 1);
        return -1;
    }
}

int32_t lds_check_meas_packet(void)
{
    uint32_t i, checksum = 0;
    
    for (i = 0; i < 20; i += 2)
    {
        checksum = (checksum << 1) + *(uint16_t *)&lds_buf[i];
    }
    checksum = (checksum + (checksum >> 15)) & 0x7FFF;
    
    if ((uint16_t)checksum == *(uint16_t *)&lds_buf[20])
    {// checksum OK
        return 0;
    }
    else
    {// checksum FAIL
        TRACE("meas packet checksum FAIL!\n");
        TRACE_ARRAY(lds_buf, 22);
        return -1;
    }
}

int32_t lds_get_packet(void)
{
    static uint8_t state = 0;
    uint8_t c;
    
    while (IS_UART5_RECV_BUF_NOT_EMPTY())
    {
        c = UART5_RX_deque();
        UART5_TX_enque(c); // pass through UART5
        switch (state)
        {
            case 0: // packet sync character
                if (c == 0xAA)
                {// LDS information
                    lds_buf[0] = 0xAA;
                    lds_buf_cnt = 1;
                    state++;
                }
                else if (c == 0xFA)
                {// LDS measurement
                    lds_buf[0] = 0xFA;
                    lds_buf_cnt = 1;
                    state = 3;
                }
                else
                {// synchronization loss
                    TRACE("sync FAIL!\n");
                }
                break;
            case 1: // LDS information packet
                if (c != 0xAA)
                {// 
                    if (c != 0xA9)
                    {
                        lds_buf[lds_buf_cnt++] = c;
                        // check packet length
                        if (lds_buf_cnt != 84 + 1)
                        {
                            
                        }
                        else
                        {// full information packet received
                            state = 0;
                            if (lds_check_info_packet() == 0)
                            {
                                return 0;
                            }
                            else
                            {
                                return -1;
                            }
                        }
                    }
                    else
                    {// 0xA9, escape character
                        lds_buf[lds_buf_cnt] = 0xA9; // don't increment lds_buf_cnt
                        state++;
                    }
                }
                else
                {// 0xAA
                    lds_buf[lds_buf_cnt++] = 0xAA;
                    TRACE("NOT expected 0xAA!\n");
                    TRACE_ARRAY(lds_buf, lds_buf_cnt);
                    state = 0;
                }
                break;
            case 2:
                if (c == 0x00)
                {// 0xA900
                    lds_buf_cnt++;
                }
                else if (c == 0x01)
                {// 0xA901
                    lds_buf[lds_buf_cnt++] = 0xAA;
                }
                else
                {// error
                    lds_buf_cnt++;
                    lds_buf[lds_buf_cnt++] = c;
                    TRACE_ARRAY(lds_buf, lds_buf_cnt);
                    state = 0;
                    break;
                }
                // check packet length
                if (lds_buf_cnt != 84 + 1)
                {
                    state = 1;
                    break;
                }
                else
                {// full information packet received
                    state = 0;
                    if (lds_check_info_packet() == 0)
                    {
                        return 0;
                    }
                    else
                    {
                        return -1;
                    }
                }
            case 3:
                lds_buf[lds_buf_cnt++] = c;
                // check packet length
                if (lds_buf_cnt != 22)
                {
                    break;
                }
                else
                {// full information packet received
                    state = 0;
                    if (lds_check_meas_packet() == 0)
                    {
                        return 0;
                    }
                    else
                    {
                        return -1;
                    }
                }
            default:
                break;
        }
    }
    
    return 1; // pending
}


void LDS_PID_Init(float Kp, float Ki, float Kd)
{
    LDS_PID.Setpoint = 300.0;
//    lds_pid.actual_speed = 0.0;
    LDS_PID.Kp = Kp;
    LDS_PID.Ki = Ki;
    LDS_PID.Kd = Kd;
    
    LDS_PID.PreError = 0.0;
    LDS_PID.Integral = 0.0;
}

float LDS_PID_Update(float speed)
{
    float error;
    float output;
    
    LDS_PID.ActualSpeed = speed;
    
    // calculate the difference between the desired value and the actual value
    error = LDS_PID.Setpoint - speed;
    
    // track error over time
    LDS_PID.Integral += error;

    // Determine the amount of change from the last time checked
    LDS_PID.Derivative = error - LDS_PID.PreError;
    
    // calculate how much drive the output in order to get to the desired set_speed.
    output = (LDS_PID.Kp * error) + (LDS_PID.Ki * LDS_PID.Integral) + (LDS_PID.Kd * LDS_PID.Derivative);
    
    if (output > 1.0f)
    {
        output = 1.0;
    }
    else if (output < 0.0f)
    {
        output = 0.0;
    }
    else
    {
    }

    // remember the error for the next time around.
    LDS_PID.PreError = error;
    
    return output;
}

void LDS_PWM_Init(void)
{
    float duty;
    
    duty = LDS_PID_Update(0);
    LDS_PWM_Update(duty);
}
    
void LDS_PWM_Update(float duty)
{
    if (duty < 0.2f && LDS_PID.ActualSpeed < 0.1f)
    {
        duty = 0.2f;
    }
    
    LL_TIM_OC_SetCompareCH3(TIM8, (LL_TIM_GetAutoReload(TIM8) + 1) * duty);
}

void lds(void)
{
    float speed;
    float duty;
    static uint32_t SpeedSum = 0;
    static uint32_t PacketCnt = 0; // measurement packet count
    
    if (lds_get_packet() == 0)
    {// ok
        if (lds_buf[0] == 0xFA)
        {// measurement packet
            SpeedSum += *(uint16_t *)&lds_buf[2];
            PacketCnt++;
        }
        else
        {// information packet
            TRACE_ARRAY(lds_buf, 84 + 1);
        }
    }
    else
    {

    }

    if (SpeedCtrlFlag)
    {
        if (PacketCnt != 0)
        {
            speed = (float)SpeedSum / (PacketCnt << 6);   // speed = SpeedSum / PacketCnt / 64
        }
        else
        {
            // use previou value
            speed = LDS_PID.ActualSpeed;
            TRACE("NO DATA!\n");
        }
        duty = LDS_PID_Update(speed);
        LDS_PWM_Update(duty);
//        TRACE5("speed = %10f duty = %f Integral = %f  Derivative = %f error = %11f\n", speed, duty, LDS_PID.Integral, LDS_PID.Derivative, LDS_PID.PreError);
//        TRACE4("speed = %10f duty = %f Integral = %f error = %11f\n", speed, duty, LDS_PID.Integral, LDS_PID.PreError);
//        TRACE3("speed = %10f duty = %f Integral = %f\n", speed, duty, LDS_PID.Integral);
//        TRACE2("speed = %10f duty = %f\n", speed, duty);
//        TRACE1("speed1 = %10f\n", speed);
        SpeedSum = 0;
        PacketCnt = 0;
        SpeedCtrlFlag = 0;
    }
}
