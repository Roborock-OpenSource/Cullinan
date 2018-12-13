
#include "config.h"
#include "stm32f4xx_ll_spi.h"
#include "BMI160.h"

struct _BMI160_REG BMI160_REG;

static uint32_t state = 0;
static uint32_t TimeStamp;


int32_t BMI160_ReadReg(uint8_t RegAddr, uint8_t *RegData)
{
    int32_t RetVal = STATE_PENDING;
    
    switch (state)
    {
        case 0:
            LL_SPI_Enable(SPI2); // tCSB_setup >= 20 ns
            if (LL_SPI_IsActiveFlag_RXNE(SPI2) == 1)
            {
                LL_SPI_ReceiveData8(SPI2); // dummy read to clear RXNE
            }
            LL_SPI_TransmitData8(SPI2, BMI160_SPI_RD_MASK | RegAddr);
            while(LL_SPI_IsActiveFlag_RXNE(SPI2) == 0) ;
            LL_SPI_ReceiveData8(SPI2); // dummy read
            LL_SPI_TransmitData8(SPI2, 0x00); // dummy write
            while(LL_SPI_IsActiveFlag_RXNE(SPI2) == 0);
            *RegData = LL_SPI_ReceiveData8(SPI2);
            
            state = 1;
            TimeStamp = LL_TIM_GetCounter(TIM2);
            LL_SPI_Disable(SPI2); // tCSB_hold >= 40 ns
            break;
        case 1:
            if (LL_TIM_GetCounter(TIM2) - TimeStamp > T_SPI_SS_DELAY)   // delay for weak pull-up, 14 us
            {
                state = 0;
                RetVal = STATE_OK;
            }
            else
            {
                
            }
            break;
        default :
            break;
    }
    
    return RetVal;
}

int32_t BMI160_ReadRegSeq(uint8_t RegAddr, uint8_t *RegData, uint8_t len)
{
    int32_t RetVal = STATE_PENDING;
    uint32_t i;
    
    switch (state)
    {
        case 0:
            LL_SPI_Enable(SPI2); // tCSB_setup >= 20 ns
            if (LL_SPI_IsActiveFlag_RXNE(SPI2) == 1)
            {
                LL_SPI_ReceiveData8(SPI2); // dummy read to clear RXNE
            }
            LL_SPI_TransmitData8(SPI2, BMI160_SPI_RD_MASK | RegAddr);
            while(LL_SPI_IsActiveFlag_RXNE(SPI2) == 0) ;
            LL_SPI_ReceiveData8(SPI2); // dummy read
            for (i = 0; i < len; i++)
            {
                LL_SPI_TransmitData8(SPI2, 0x00); // dummy write
                while(LL_SPI_IsActiveFlag_RXNE(SPI2) == 0);
                *RegData++ = LL_SPI_ReceiveData8(SPI2);
            }
            state = 1;
            TimeStamp = LL_TIM_GetCounter(TIM2);
            LL_SPI_Disable(SPI2); // tCSB_hold >= 40 ns
            break;
        case 1:
            if (LL_TIM_GetCounter(TIM2) - TimeStamp > T_SPI_SS_DELAY)   // delay for weak pull-up, 14 us
            {
                state = 0;
                RetVal = STATE_OK;
            }
            else
            {
                
            }
            break;
        default :
            break;
    }
    
    return RetVal;
}


int32_t BMI160_WriteReg(uint8_t RegAddr, uint8_t RegData)
{
    int32_t RetVal = STATE_PENDING;
    static uint32_t WaitTime;
    
    switch (state)
    {
        case 0:
            LL_SPI_Enable(SPI2);
            if (LL_SPI_IsActiveFlag_RXNE(SPI2) == 1)
            {
                LL_SPI_ReceiveData8(SPI2); // dummy
            }
            LL_SPI_TransmitData8(SPI2, BMI160_SPI_WR_MASK & RegAddr);
            while(LL_SPI_IsActiveFlag_RXNE(SPI2) == 0);
            LL_SPI_ReceiveData8(SPI2); // dummy
            LL_SPI_TransmitData8(SPI2, RegData);
            while(LL_SPI_IsActiveFlag_RXNE(SPI2) == 0);
            LL_SPI_ReceiveData8(SPI2); // dummy

            state = 1;
            TimeStamp = LL_TIM_GetCounter(TIM2);
            if (BMI160_REG.PMU_STATUS.acc_pmu_status == BMI160_ACC_PMU_SUSPEND
                && BMI160_REG.PMU_STATUS.gyr_pmu_status == BMI160_GYR_PMU_SUSPEND)
            {
                WaitTime = T_SPI_SS_DELAY + T_IDLE_WACC_SUM;
            }
            else
            {
                WaitTime = T_SPI_SS_DELAY;
            }
            LL_SPI_Disable(SPI2); // tCSB_hold >= 40 ns
            break;
        case 1:
            if (LL_TIM_GetCounter(TIM2) - TimeStamp > WaitTime)
            {
                state = 0;
                RetVal = STATE_OK;
            }
            else
            {
                
            }
            break;
        default :
            break;
    }
    
    return RetVal;
}

// multiple write
int32_t BMI160_WriteRegSeq(uint8_t RegAddr, uint8_t *RegData, uint8_t len)
{
    int32_t RetVal = STATE_PENDING;
    static uint32_t WaitTime;
    static uint32_t i;
    
    switch (state)
    {
        case 0:
            if (BMI160_REG.PMU_STATUS.acc_pmu_status == BMI160_ACC_PMU_NORMAL
                || BMI160_REG.PMU_STATUS.gyr_pmu_status == BMI160_GYR_PMU_NORMAL)
            {
                state = 1;
            }
            else
            {
                i = 0;
                state = 3;
                goto state3;
            }
        case 1:
            LL_SPI_Enable(SPI2);
            if (LL_SPI_IsActiveFlag_RXNE(SPI2) == 1)
            {
                LL_SPI_ReceiveData8(SPI2); // dummy
            }
            LL_SPI_TransmitData8(SPI2, BMI160_SPI_WR_MASK & RegAddr);
            while(LL_SPI_IsActiveFlag_RXNE(SPI2) == 0);
            LL_SPI_ReceiveData8(SPI2); // dummy
            for (i = 0; i < len; i++)
            {
                LL_SPI_TransmitData8(SPI2, *RegData++); // burst write in normal mode
                while(LL_SPI_IsActiveFlag_RXNE(SPI2) == 0);
                LL_SPI_ReceiveData8(SPI2); // dummy
            }
            state = 2;
            TimeStamp = LL_TIM_GetCounter(TIM2);
            WaitTime = T_SPI_SS_DELAY;
            LL_SPI_Disable(SPI2); // tCSB_hold >= 40 ns
            break;
        case 2:
            if (LL_TIM_GetCounter(TIM2) - TimeStamp > WaitTime)
            {
                state = 0;
                RetVal = STATE_OK;
            }
            else
            {
                break;
            }
        case 3: state3:
            // Burst write is not allowed in suspend or low power mode
            LL_SPI_Enable(SPI2);
            if (LL_SPI_IsActiveFlag_RXNE(SPI2) == 1)
            {
                LL_SPI_ReceiveData8(SPI2); // dummy
            }
            LL_SPI_TransmitData8(SPI2, BMI160_SPI_WR_MASK & (RegAddr + i));
            while(LL_SPI_IsActiveFlag_RXNE(SPI2) == 0);
            LL_SPI_ReceiveData8(SPI2); // dummy
            LL_SPI_TransmitData8(SPI2, *(RegData + i));
            while(LL_SPI_IsActiveFlag_RXNE(SPI2) == 0);
            LL_SPI_ReceiveData8(SPI2); // dummy
            state = 4;
            TimeStamp = LL_TIM_GetCounter(TIM2);
            WaitTime = T_SPI_SS_DELAY + T_IDLE_WACC_SUM;
            LL_SPI_Disable(SPI2); // tCSB_hold >= 40 ns
            break;
        case 4:
            if (LL_TIM_GetCounter(TIM2) - TimeStamp > WaitTime)
            {
                if (++i < len)
                {
                    state = 3;
                    goto state3;
                }
                else
                {
                    state = 0;
                    RetVal = STATE_OK;
                }
            }
            else
            {
                
            }
            break;
        default :
            break;
    }
    
    return RetVal;
}
