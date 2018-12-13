
#include "config.h"
#include "BMI160.h"
#include "usbd_cdc_if.h"


extern volatile uint32_t IMUFlag;


void IMU(void)
{
    static uint32_t state;
    int32_t RetVal;
    
    switch (state)
    {
        case 0:
            /* Select the Output data rate, range of accelerometer sensor */
            BMI160_REG.ACC_CONF.acc_odr = BMI160_ACCEL_ODR_100HZ;
            BMI160_REG.ACC_CONF.acc_bwp = BMI160_ACCEL_BW_NORMAL_AVG4;
            BMI160_REG.ACC_CONF.acc_us = 0;
            BMI160_REG.ACC_RANGE.acc_range = BMI160_ACCEL_RANGE_2G;
        
            /* Select the Output data rate, range of Gyroscope sensor */
            BMI160_REG.GYR_CONF.gyr_odr = BMI160_GYRO_ODR_100HZ;
            BMI160_REG.GYR_CONF.gyr_bwp = BMI160_GYRO_BW_NORMAL_MODE;
            BMI160_REG.GYR_RANGE.gyr_range = BMI160_GYRO_RANGE_2000_DPS;
        
            /* Configure interrupt */
            BMI160_REG.INT_OUT_CTRL.int1_lvl = 1; // Choosing active high output
            BMI160_REG.INT_OUT_CTRL.int1_od = 0; // Choosing push-pull mode for interrupt pin
            BMI160_REG.INT_OUT_CTRL.int1_output_en = 1; // enable interrupt output
        
            BMI160_REG.INT_MAP.int1_drdy = 1; // map data ready interrupt to INT1 pin
            BMI160_REG.INT_EN.int_drdy_en = 1; // enable data ready interrupt
        
            state = 1;
        case 1:
            RetVal = BMI160_WriteRegSeq(REG(ACC_CONF), (uint8_t *)&BMI160_REG.ACC_CONF, 2); /* Write ACC_CONF and ACC_RANGE */
            if (RetVal == STATE_OK)
            {
                state = 2;
            }
            else
            {
                break;
            }
        case 2:
            RetVal = BMI160_WriteRegSeq(REG(GYR_CONF), (uint8_t *)&BMI160_REG.GYR_CONF, 2); /* Write GYR_CONF and GYR_RANGE */
            if (RetVal == STATE_OK)
            {
                state = 3;
            }
            else
            {
                break;
            }
        case 3:
            RetVal = BMI160_WriteReg(REG(INT_OUT_CTRL), BMI160_REG.INT_OUT_CTRL.all);
            if (RetVal == STATE_OK)
            {
                state = 4;
            }
            else
            {
                break;
            }
        case 4:
            RetVal = BMI160_WriteReg(REG(INT_MAP) + 1, BMI160_REG.INT_MAP.all[1]);
            if (RetVal == STATE_OK)
            {
                state = 5;
            }
            else
            {
                break;
            }
        case 5:
            RetVal = BMI160_WriteReg(REG(INT_EN) + 1, BMI160_REG.INT_EN.all[1]);
            if (RetVal == STATE_OK)
            {
                state = 6;
            }
            else
            {
                break;
            }
        case 6:
            RetVal = BMI160_SetAccelNormal();
            if (RetVal == STATE_OK)
            {
                state = 7;
            }
            else
            {
                break;
            }
        case 7:
            RetVal = BMI160_SetGyroNormal();
            if (RetVal == STATE_OK)
            {
                IMUFlag = 0;
                state = 8;
            }
            else
            {
                break;
            }
        case 8:
            if (IMUFlag)
            {
                IMUFlag = 0;
                state = 9;
            }
            else
            {
                break;
            }
        case 9:
            RetVal = BMI160_ReadRegSeq(REG(DATA.gyr_x_7_0), (uint8_t *)&BMI160_REG.DATA.gyr_x_7_0, 12 + 3);
            if (RetVal == STATE_OK)
            {
                state = 10;
            }
            else
            {
                break;
            }
        case 10:
            RetVal = CDC_Transmit_FS((uint8_t *)&BMI160_REG.DATA.gyr_x_7_0, 12 + 3);
            if (RetVal == USBD_OK)
            {
                state = 8;
            }
            else if (RetVal == USBD_BUSY)
            {
                
            }
            else /* if (RetVal == USBD_FAIL) */
            {
                
            }
            break;
        default :
            break;
    }
}






