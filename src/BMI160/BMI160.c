

#include "config.h"
#include "BMI160.h"



static uint32_t state = 0;
uint32_t BMI160_Timer = 0;


const float FS_2G_LSB_PER_G = 16384.0f;
const float FS_4G_LSB_PER_G = 8192.0f;
const float FS_8G_LSB_PER_G = 4096.0f;
const float FS_16G_LSB_PER_G = 2048.0f;


const float FS_2000_LSB_PER_DPS = 16.4f;
const float FS_1000_LSB_PER_DPS = 32.8f;
const float FS_500_LSB_PER_DPS = 65.6f;
const float FS_250_LSB_PER_DPS = 131.2f;
const float FS_125_LSB_PER_DPS = 262.4f;

/*!
 *  @brief This API is the entry point for sensor.It performs
 *  the selection of SPI read mechanism and reads the chip-id of bmi160 sensor.
 */
int32_t BMI160_Init(void)
{
    int32_t RetVal = STATE_PENDING;
    static uint32_t TryCnt;
        
    switch (state)
    {
        case 0:
            /* Dummy read of 0x7F register to enable SPI Interface */
            RetVal = BMI160_ReadReg(REG(SPI_COMM_TEST), &BMI160_REG.SPI_COMM_TEST);
            if (RetVal == STATE_OK)
            {
                state = 1;
                TryCnt = 3;
            }
            else
            {
                break;
            }
        case 1: state1:
            RetVal = BMI160_ReadReg(REG(CHIP_ID), &BMI160_REG.CHIP_ID); /* Read chip_id */
            if (RetVal == STATE_OK)
            {
                if (BMI160_REG.CHIP_ID == BMI160_CHIP_ID)
                {
                    state = 2;
                }
                else
                {
                    if (--TryCnt == 0)
                    {
                        RetVal = -1; // device not found
                        state = 0;
                        break;
                    }
                    else
                    {
                        goto state1; // try again
                    }
                }
            }
            else
            {
                break;
            }
        case 2:
            RetVal = BMI160_SoftReset(); /* Soft reset */
            if (RetVal == STATE_OK)
            {
                state = 0;
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


/*!
 * @brief This API resets and restarts the device.
 * All register values are overwritten with default parameters.
 */
int32_t BMI160_SoftReset(void)
{
    int32_t RetVal = STATE_PENDING;
    static uint32_t state;
    
    switch (state)
    {
        case 0:
            /* Reset the device */
            RetVal = BMI160_WriteReg(REG(CMD), BMI160_SOFT_RESET_CMD);
            if (RetVal == STATE_OK)
            {
                BMI160_Timer = BMI160_SOFT_RESET_DELAY_MS;
                state = 1;
                RetVal = STATE_PENDING;
            }
            else
            {
                
            }
            break;
        case 1:
            if (BMI160_Timer == 0)
            {
                state = 2;
                // go to next state
            }
            else
            {
                break;
            }
        case 2:
            /* Dummy read of 0x7F register to enable SPI Interface */
            RetVal = BMI160_ReadReg(REG(SPI_COMM_TEST), &BMI160_REG.SPI_COMM_TEST);
            if (RetVal == STATE_OK)
            {
                /* Update the default parameters */
//                default_param_settg(dev);
                state = 0;
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


/*!
 * @brief This API sets the accel normal power.
 */
int32_t BMI160_SetAccelNormal(void)
{
    int32_t RetVal = STATE_PENDING;
    
    switch (state)
    {
        case 0:
            RetVal = BMI160_ReadReg(REG(PMU_STATUS), (uint8_t *)&BMI160_REG.PMU_STATUS);
            if (RetVal == STATE_OK)
            {
                if (BMI160_REG.PMU_STATUS.acc_pmu_status != BMI160_ACC_PMU_NORMAL)
                {
                    state = 1;
                    // process_under_sampling
                }
                else
                {// already in normal mode
                    break;
                }
            }
            else
            {
                break;
            }
        case 1:
            RetVal = BMI160_WriteReg(REG(CMD), BMI160_ACCEL_NORMAL_MODE); /* Write accel power */
            if (RetVal == STATE_OK)
            {
                if (BMI160_REG.PMU_STATUS.acc_pmu_status == BMI160_ACC_PMU_SUSPEND)
                {
                    BMI160_Timer = BMI160_ACCEL_DELAY_MS; /* Add delay of 5 ms */
                    state = 2;
                    RetVal = STATE_PENDING;
                }
                else
                {
                    state = 3;
                    goto state3;
                }
            }
            else
            {
                
            }
            break;
        case 2:
            if (BMI160_Timer == 0)
            {
                state = 3;
            }
            else
            {
                break;
            }
        state3:
        case 3:
            RetVal = BMI160_ReadReg(REG(PMU_STATUS), (uint8_t *)&BMI160_REG.PMU_STATUS);
            if (RetVal == STATE_OK)
            {
                state = 0;
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

/*!
 * @brief This API sets the gyro power mode.
 */
int32_t BMI160_SetGyroNormal(void)
{
    int32_t RetVal = STATE_PENDING;

    switch (state)
    {
        case 0:
            RetVal = BMI160_ReadReg(BMI160_PMU_STATUS_ADDR, (uint8_t *)&BMI160_REG.PMU_STATUS);
            if (RetVal == STATE_OK)
            {
                if (BMI160_REG.PMU_STATUS.gyr_pmu_status != BMI160_GYR_PMU_NORMAL)
                {
                    state = 1;
                }
                else
                {// already in normal mode
                    break;
                }
            }
            else
            {
                break;
            }
        case 1:
            RetVal = BMI160_WriteReg(BMI160_COMMAND_REG_ADDR, BMI160_GYRO_NORMAL_MODE); /* Write gyro power */
            if (RetVal == STATE_OK)
            {
                if (BMI160_REG.PMU_STATUS.gyr_pmu_status == BMI160_GYR_PMU_SUSPEND)
                {
                    BMI160_Timer = BMI160_GYRO_DELAY_MS; /* Add delay of 81 ms */
                    state = 2;
                    RetVal = STATE_PENDING;
                }
                else if (BMI160_REG.PMU_STATUS.gyr_pmu_status == BMI160_GYR_PMU_FAST_STARTUP)
                {
                    BMI160_Timer = BMI160_GYRO_F2N_DELAY_MS; /* This delay is required for transition from fast-startup mode to normal mode */
                    state = 2;
                    RetVal = STATE_PENDING;
                }
                else
                {
                    state = 0;
                }
            }
            else
            {
                
            }
            break;
        case 2:
            if (BMI160_Timer == 0)
            {
                state = 3;
            }
            else
            {
                break;
            }
        case 3:
            RetVal = BMI160_ReadReg(BMI160_PMU_STATUS_ADDR, (uint8_t *)&BMI160_REG.PMU_STATUS);
            if (RetVal == STATE_OK)
            {
                state = 0;
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
