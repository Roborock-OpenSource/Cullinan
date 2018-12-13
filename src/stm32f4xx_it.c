/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "config.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_exti.h"

extern uint8_t CmdRecvBuf[CMD_RECV_BUF_SIZE];
extern uint8_t *CmdRecvHead;
extern uint8_t *CmdRecvTail;
extern uint8_t CmdSendBuf[CMD_SEND_BUF_SIZE];
extern uint8_t *CmdSendHead;
extern uint8_t * volatile CmdSendTail;


uint8_t UART5_RecvBuf[UART5_RECV_BUF_SIZE];
uint8_t UART5_SendBuf[UART5_SEND_BUF_SIZE];

uint8_t *UART5_RecvHead;
const uint8_t *UART5_RecvTail;
uint8_t *UART5_SendHead;
const uint8_t * volatile UART5_SendTail;

volatile uint32_t SysTimer;
uint32_t SpeedTimer;
uint32_t SpeedCtrlFlag;

volatile uint32_t IMUFlag;
extern uint32_t BMI160_Timer;
/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
    HAL_IncTick();
    
    if (SysTimer != 0)
    {
        SysTimer--;
    }
    if (--SpeedTimer == 0)
    {
        SpeedTimer = 10;
        SpeedCtrlFlag = 1;
    }
    if (BMI160_Timer != 0)
    {
        BMI160_Timer--;
    }
    
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
    /* Check RXNE flag value in SR register */
    if (LL_USART_IsActiveFlag_RXNE(USART3))
    {
        /* Read Received character. RXNE flag is cleared by reading of DR register */
        *CmdRecvHead = LL_USART_ReceiveData8(USART3);
        if(CmdRecvHead == CmdRecvBuf + CMD_RECV_BUF_SIZE - 1)
        {
            CmdRecvHead = &CmdRecvBuf[0];
        }
        else
        {
            CmdRecvHead++;
        }
    }
    
    /* check TXE flag */
    if (LL_USART_IsEnabledIT_TXE(USART3) && LL_USART_IsActiveFlag_TXE(USART3))
    {
        if (CmdSendTail != CmdSendHead) /* check if send buffer is empty or not */
        {// not empty
            /* Fill DR with a new char and clear TXE */
            LL_USART_TransmitData8(USART3, *CmdSendTail);
            if(CmdSendTail == CmdSendBuf + CMD_SEND_BUF_SIZE - 1)
            {
                CmdSendTail = &CmdSendBuf[0];
            }
            else
            {
                CmdSendTail++;
            }
        }
        else
        {// empty
            LL_USART_DisableIT_TXE(USART3);
        }
    }
}

/**
  * @brief This function handles UART5 global interrupt.
  */
void UART5_IRQHandler(void)
{
    /* Check RXNE flag value in SR register */
    if (LL_USART_IsActiveFlag_RXNE(UART5))// && LL_USART_IsEnabledIT_RXNE(UART5))
    {
//        if (LL_USART_IsActiveFlag_FE(UART5) || LL_USART_IsActiveFlag_NE(UART5) || LL_USART_IsActiveFlag_ORE(UART5))
//        {
//            __nop();
//        }
        
        /* Read Received character. RXNE flag is cleared by reading of DR register */
        *UART5_RecvHead = LL_USART_ReceiveData8(UART5);
        if(UART5_RecvHead == UART5_RecvBuf + UART5_RECV_BUF_SIZE - 1)
        {
            
//            LL_USART_DisableIT_RXNE(UART5);
            
            
            UART5_RecvHead = &UART5_RecvBuf[0];
        }
        else
        {
            UART5_RecvHead++;
        }
        
//        while(UART5_RecvHead == UART5_RecvTail)  // check if receive buffer full
//        {// buffer full
//            // receive buffer overflow!
//        }
    }
    
    /* check TXE flag */
    if (LL_USART_IsEnabledIT_TXE(UART5) && LL_USART_IsActiveFlag_TXE(UART5))
    {
        if (UART5_SendTail != UART5_SendHead) /* check if send buffer is empty or not */
        {// not empty
            /* Fill DR with a new char and clear TXE */
            LL_USART_TransmitData8(UART5, *UART5_SendTail);
            if(UART5_SendTail == UART5_SendBuf + UART5_SEND_BUF_SIZE - 1)
            {
                UART5_SendTail = &UART5_SendBuf[0];
            }
            else
            {
                UART5_SendTail++;
            }
        }
        else
        {// empty
            LL_USART_DisableIT_TXE(UART5);
        }
    }
}

/**
  * @brief  This function handles external line 9 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI9_5_IRQHandler(void)
{
  /* Manage Flags */
  if(LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_9) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_9);
    IMUFlag = 1;
  }
}
/**
  * @brief This function handles USB On The Go FS global interrupt.
  */
void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */

  /* USER CODE END OTG_FS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
