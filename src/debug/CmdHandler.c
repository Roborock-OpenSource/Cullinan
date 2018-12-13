
#include "config.h"
#include <string.h>
#include <float.h>
#include "stm32f4xx.h"   // for __INLINE
#include "LDS.h"
#include "stm32f4xx_ll_tim.h"
#include "BMI160.h"
#include "stm32f4xx_ll_utils.h"
#include "usbd_cdc_if.h"

extern uint8_t CmdLineBuf[CMD_LINE_SIZE];

extern uint8_t lds_buf[LDS_BUF_SIZE];
extern volatile uint32_t SysTimer;
extern uint32_t SpeedCtrlFlag;

uint32_t DebugOn;

void meas(void);
void rd160(void);
void wr160(void);
void dump160(void);
void set160(void);
void data160(void);
void temp160(void);
void tune(void);
void kc(void);
void t(void);

__STATIC_INLINE void MakeLower(uint8_t *p)
{
    for(; *p; p++)
    {
        if(*p >= 'A' && *p <= 'Z')
        {
            *p += 0x20;
        }
    }
}

void CmdHandler(void)
{
    uint8_t CommandName[CMD_NAME_SIZE];
    
    if(sscanf((char *)CmdLineBuf, "%s", CommandName) == 1)
    {
        MakeLower(CommandName);
        if(strcmp((char *)CommandName, "debugon") == 0)
        {
            DebugOn = 1;
        }
        else if(strcmp((char *)CommandName, "debugoff") == 0)
        {
            DebugOn = 0;
        }
        else if(strcmp((char *)CommandName, "meas") == 0)
        {
            meas();
        }
        else if(strcmp((char *)CommandName, "rd160") == 0)
        {
            rd160();
        }
        else if(strcmp((char *)CommandName, "wr160") == 0)
        {
            wr160();
        }
        else if(strcmp((char *)CommandName, "dump160") == 0)
        {
            dump160();
        }
        else if(strcmp((char *)CommandName, "set160") == 0)
        {
            set160();
        }
        else if(strcmp((char *)CommandName, "data160") == 0)
        {
            data160();
        }
        else if(strcmp((char *)CommandName, "temp160") == 0)
        {
            temp160();
        }
        else if(strcmp((char *)CommandName, "tune") == 0)
        {
            tune();
        }
        else if(strcmp((char *)CommandName, "kc") == 0)
        {
            kc();
        }
        else if(strcmp((char *)CommandName, "t") == 0)
        {
            t();
        }
        else
        {
            TRACE("Bad command!\n");
        }
    }
    else
    {
        TRACE("Bad command prompt!\n");
    }
}

void meas(void)
{
    uint8_t index = 0;

    while(1)
    {
        if (lds_get_packet() == 0)
        {// ok
            if (lds_buf[0] == 0xFA)
            {// measurement packet
                if (lds_buf[1] >= 0xA0 && lds_buf[1] <= 0xF9)
                {// valid index
                    if (index == 0)
                    {
                        index = lds_buf[1] + 1;
                        if (index > 0xF9)
                        {
                            index = 0xA0;
                        }
                    }
                    else
                    {
                        if (index != lds_buf[1])
                        {
                            TRACE("index SYNC FAIL!\n");
                        }
                        else
                        {
                            index++;
                            if (index > 0xF9)
                            {
                                index = 0xA0;
                            }
                        }
                    }
                    TRACE1("speed = %10f\n", (float)*(uint16_t *)&lds_buf[2] / 64);
                }
                else
                {
                    TRACE("index FAIL!\n");
                    TRACE_ARRAY(lds_buf, 22);
                }
            }
            else
            {// information packet
                TRACE_ARRAY(lds_buf, 84 + 1);
            }
        }
        else
        {
            
        }
    }
}

void lds_test(void)
{
    uint32_t i, j;
    uint32_t duty;
    uint32_t sum;
    
    for (j = 0; j <= 100; j++)
    {
        duty = ((LL_TIM_GetAutoReload(TIM8) + 1) * j + 50) / 100;
        LL_TIM_OC_SetCompareCH3(TIM8, duty);

        SysTimer = 10000;
        while (SysTimer != 0) ;
        
        
        SysTimer = 1000;
        
        for (sum = 0, i = 0; i < 100; )
        {
            if (lds_get_packet() == 0)
            {// ok
                if (lds_buf[0] == 0xFA)
                {// measurement packet
                    if (lds_buf[1] >= 0xA0 && lds_buf[1] <= 0xF9)
                    {// valid index
                        sum += *(uint16_t *)&lds_buf[2];
                        i++;
                    }
                    else
                    {// invalid
                        TRACE1("INVALID index = %02X\n", lds_buf[1]);
                        return;
                    }
                }
                else
                {// 
                }
            }
            else
            {
                if (SysTimer == 0 && i == 0)
                {
                    break;
                }
            }
        }
        if (i != 0)
        {
            TRACE3("duty = %3u speed = %10.6f i = %3u\n", j, (float)sum / (64 * i), i);
        }
        else
        {
            TRACE3("duty = %3u speed = %10.6f i = %3u\n", j, (float)sum, i);
        }
        
        LL_TIM_OC_SetCompareCH3(TIM8, 0); // stop
        SysTimer = 10000;
        while (SysTimer != 0) ;
    }
    
    while(1);
}

void rd160(void)
{
    uint8_t RegAddr = 0;
    uint8_t RegData;
    int32_t RetVal;
    
    if (sscanf((char *)CmdLineBuf, "%*s %hhx", &RegAddr) == 1)
    {
        while ((RetVal = BMI160_ReadReg(RegAddr, &RegData)) == STATE_PENDING) ;
        if(RetVal == 0)
        {
            TRACE2("%02X=%02X\n", RegAddr, RegData);
        }
        else
        {
            
        }
    }
    else
    {
        TRACE("Bad parameter! Please use:\n");
        TRACE("rd160 reg_addr\n");
    }
}

void wr160(void)
{
    uint8_t RegAddr = 0;
    uint8_t RegData;
    int8_t RetVal;
    
    if (sscanf((char *)CmdLineBuf, "%*s %hhx %hhx", &RegAddr, &RegData) == 2)
    {
        while ((RetVal = BMI160_WriteReg(RegAddr, RegData)) == STATE_PENDING) ;
        if(RetVal == 0)
        {
            while ((RetVal = BMI160_ReadReg(RegAddr, &RegData)) == STATE_PENDING) ;
            if(RetVal == 0)
            {
                TRACE2("read back %02X=%02X\n", RegAddr, RegData);
            }
            else
            {
                
            }
        }
        else
        {
            
        }
    }
    else
    {
        TRACE("Bad parameter! Please use:\n");
        TRACE("wr160 reg_addr reg_data\n");
    }
}

void dump160(void)
{
    uint8_t i;
    uint8_t buf[128];

    TRACE("     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");
    while (BMI160_ReadRegSeq(0x00, &buf[0], 0x24) == STATE_PENDING) ;
    // see datasheet section 2.2.1
    buf[0x24] = 0; // 0x24
    while (BMI160_ReadRegSeq(0x25, &buf[0x25], 0x5B) == STATE_PENDING) ;
    for (i = 0; i < 128; i += 16)
    {
        TRACE1("[%02X]:", i);
        TRACE_ARRAY(&buf[i], 16);
    }
}

void set160(void)
{
    BMI160_REG.INT_OUT_CTRL.int1_lvl = 1; // Choosing active high output
    BMI160_REG.INT_OUT_CTRL.int1_od = 0; // Choosing push-pull mode for interrupt pin
    BMI160_REG.INT_OUT_CTRL.int1_output_en = 1; // enable interrupt output
    while (BMI160_WriteReg(BMI160_INT_OUT_CTRL_ADDR, BMI160_REG.INT_OUT_CTRL.all) == STATE_PENDING) ;
    
    BMI160_REG.INT_MAP.int1_drdy = 1; // map data ready interrupt to INT1 pin
    while (BMI160_WriteReg(BMI160_INT_MAP_1_ADDR, BMI160_REG.INT_MAP.all[1]) == STATE_PENDING) ;
    
    BMI160_REG.INT_EN.int_drdy_en = 1; // enable data ready interrupt
    while (BMI160_WriteReg(BMI160_INT_ENABLE_1_ADDR, BMI160_REG.INT_EN.all[1]) == STATE_PENDING) ;
    
    /* Select the Output data rate, range of accelerometer sensor */
    BMI160_REG.ACC_CONF.acc_odr = BMI160_ACCEL_ODR_100HZ;
    BMI160_REG.ACC_CONF.acc_bwp = BMI160_ACCEL_BW_NORMAL_AVG4;
    BMI160_REG.ACC_CONF.acc_us = 0;
    BMI160_REG.ACC_RANGE.acc_range = BMI160_ACCEL_RANGE_2G;
    while (BMI160_WriteRegSeq(REG(ACC_CONF), (uint8_t *)&BMI160_REG.ACC_CONF, 2) == STATE_PENDING) ;
    while (BMI160_SetAccelNormal() == STATE_PENDING) ; // set normal power mode
    
    /* Select the Output data rate, range of Gyroscope sensor */
    BMI160_REG.GYR_CONF.gyr_odr = BMI160_GYRO_ODR_100HZ;
    BMI160_REG.GYR_CONF.gyr_bwp = BMI160_GYRO_BW_NORMAL_MODE;
    BMI160_REG.GYR_RANGE.gyr_range = BMI160_GYRO_RANGE_2000_DPS;
    while (BMI160_WriteRegSeq(REG(GYR_CONF), (uint8_t *)&BMI160_REG.GYR_CONF, 2) == STATE_PENDING) ;
    while (BMI160_SetGyroNormal() == STATE_PENDING) ; // set normal power mode
}

void data160(void)
{
    while (1)
    {
        while (BMI160_ReadReg(BMI160_STATUS_ADDR, (uint8_t *)&BMI160_REG.STATUS) == STATE_PENDING) ;
        if (BMI160_REG.STATUS.drdy_acc)
        {
            while (BMI160_ReadRegSeq(BMI160_GYRO_DATA_ADDR, (uint8_t *)&BMI160_REG.DATA.gyr_x_7_0, 12 + 3) == STATE_PENDING) ;
            TRACE3("ACC.x=%10f ACC.y=%10f ACC.z=%10f", (float)*(int16_t *)&BMI160_REG.DATA.acc_x_7_0 / FS_2G_LSB_PER_G
            , (float)*(int16_t *)&BMI160_REG.DATA.acc_y_7_0 / FS_2G_LSB_PER_G
            , (float)*(int16_t *)&BMI160_REG.DATA.acc_z_7_0 / FS_2G_LSB_PER_G);
        
            TRACE3(" GYR.x=%10f GYR.y=%10f GYR.z=%10f", (float)*(int16_t *)&BMI160_REG.DATA.gyr_x_7_0 / FS_2000_LSB_PER_DPS
            , (float)*(int16_t *)&BMI160_REG.DATA.gyr_y_7_0 / FS_2000_LSB_PER_DPS
            , (float)*(int16_t *)&BMI160_REG.DATA.gyr_z_7_0 / FS_2000_LSB_PER_DPS);
            
            TRACE1(" @ %8u\n", ((uint32_t)BMI160_REG.SENSORTIME.sensor_time_23_16 << 16) | ((uint32_t)BMI160_REG.SENSORTIME.sensor_time_15_8 << 8) | (uint32_t)BMI160_REG.SENSORTIME.sensor_time_7_0);
        }
    }
}

void temp160(void)
{
    while (1)
    {
        while (BMI160_ReadRegSeq(0x20, (uint8_t *)&BMI160_REG.TEMPERATURE.temperature_7_0, 2) == STATE_PENDING) ;
        TRACE1("T=%10f\n", (*(int16_t *)&BMI160_REG.TEMPERATURE.temperature_7_0 + 23 * 512) * (1.0f / 512));
        LL_mDelay(100);
    }
}

// Kc tuning
// Kc = 0.0082, Pc = 0.46s
// 
void kc(void)
{
    uint32_t i;
    float speed;
    float duty;
    float SetSpeed = 300.0; // rpm
    float Kp;
    float KpMax = 0.01;
    float KpMin = 0.001;
    uint32_t KpStep = 5;
    float error;
    uint32_t PacketCnt; // measurement packet count
    uint32_t SpeedSum;
    
    for (i = 0; i <= KpStep; i++)
    {
        Kp = KpMin + (KpMax - KpMin) / KpStep * i;
        TRACE1("*** Kp = %f ***\n", Kp);
        SysTimer = 12000;   // 12 s
        SpeedSum = 0;
        PacketCnt = 0;
        do
        {
            if (lds_get_packet() == 0)
            {// ok
                if (lds_buf[0] == 0xFA)
                {// measurement packet
                    if (lds_buf[1] >= 0xA0 && lds_buf[1] <= 0xF9)
                    {// valid index
                        SpeedSum += *(uint16_t *)&lds_buf[2];
                        PacketCnt++;
                    }
                    else
                    {
                        TRACE("index FAIL!\n");
                        TRACE_ARRAY(lds_buf, 22);
                    }
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
                    speed = 0;
                }
                error = SetSpeed - speed;
                duty = Kp * error;
                if (duty > 1.0f)
                {
                    duty = 1.0f;
                }
                else if (duty < 0.0f)
                {
                    duty = 0.0f;
                }
                else
                {
                    // keep
                }
                LL_TIM_OC_SetCompareCH3(TIM8, (LL_TIM_GetAutoReload(TIM8) + 1) * duty);
                TRACE3("speed = %10f error = %11f duty = %f\n", speed, error, duty);
                SpeedCtrlFlag = 0;
                SpeedSum = 0;
                PacketCnt = 0;
            }
        } while (SysTimer != 0);

        LL_TIM_OC_SetCompareCH3(TIM8, 0); // stop
        SysTimer = 5000;
        while (SysTimer != 0) ;
        TRACE("\n\n");
    }
}

// pid tuning
// Kc = 0.0082, Pc = 0.46s
// 
void tune(void)
{
    uint32_t i, j, n;
    float speed;
    float duty;

    float Dt = 0.01; // sample time 10 ms
    
    float Kc = 0.0082;
    float Pc = 0.46;
    float tolerance = 0.1; // +-10%
    
    float Kp = 0.6f * Kc;
    float KpMax = (1 + tolerance) * Kp;
    float KpMin = (1 - tolerance) * Kp;
    uint32_t KpStep = 1;
    
    float Ki;
    float Ti = 0.5f * Pc;
    float TiMax = (1 + tolerance) * Ti;
    float TiMin = (1 - tolerance) * Ti;
    uint32_t TiStep = 1;
    
    float Kd;
    float Td = 0.125f * Pc;
    float TdMax = (1 + tolerance) * Td;
    float TdMin = (1 - tolerance) * Td;
    uint32_t TdStep = 1;
    
    uint32_t PacketCnt; // measurement packet count
    uint32_t SpeedSum;
    
    for (i = 0; i <= KpStep; i++)
    {
        Kp = KpMin + (KpMax - KpMin) / KpStep * i;
        for (j = 0; j <= TiStep; j++)
        {
            Ti = TiMin + (TiMax - TiMin) / TiStep * j;
            Ki = Kp / Ti * Dt;
            for (n = 0; n <= TdStep; n++)
            {
                Td = TdMin + (TdMax - TdMin) / TdStep * n;
                Kd = Kp * Td / Dt;
                TRACE3("*** Kp = %f Ki = %f Kd = %f ***\n", Kp, Ki, Kd);
                LDS_PID_Init(Kp, Ki, Kd);
                SysTimer = 10000;   // 10 s
                SpeedSum = 0;
                PacketCnt = 0;
                UART5_RX_SetEmpty();
                do
                {
                    if (lds_get_packet() == 0)
                    {// ok
                        if (lds_buf[0] == 0xFA)
                        {// measurement packet
                            if (lds_buf[1] >= 0xA0 && lds_buf[1] <= 0xF9)
                            {// valid index
                                SpeedSum += *(uint16_t *)&lds_buf[2];
                                PacketCnt++;
                            }
                            else
                            {
                                TRACE("index FAIL!\n");
                                TRACE_ARRAY(lds_buf, 22);
                            }
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
                            speed = 0;
                        }
                        duty = LDS_PID_Update(speed);
                        LL_TIM_OC_SetCompareCH3(TIM8, (LL_TIM_GetAutoReload(TIM8) + 1) * duty);
                        TRACE2("speed = %10f duty = %f\n", speed, duty);
                        SpeedCtrlFlag = 0;
                        SpeedSum = 0;
                        PacketCnt = 0;
                    }
                } while (SysTimer != 0);

                LL_TIM_OC_SetCompareCH3(TIM8, 0); // stop
                SysTimer = 10000;
                while (SysTimer != 0) ;
                TRACE("\n\n");
            }
        }
    }
}

// SPI single read test
void t11(void)
{
    uint32_t i;
    uint8_t reg;
    
    while (BMI160_ReadReg(0x00, &reg) == STATE_PENDING) ; // dummy read
    for (i = 0; i < 10000; i++)
    {
        reg = 0;
        while (BMI160_ReadReg(0x00, &reg) == STATE_PENDING) ;
        if (reg != BMI160_CHIP_ID)
        {
            TRACE1("t11 FAIL %02X\n", reg);
            return;
        }
        else
        {
            // continue
        }
    }
    TRACE("t11 PASS\n");
}

// SPI sequencial read test
void t12(void)
{
    uint32_t i;
    uint8_t reg[3];
    
    for (i = 0; i < 10000; i++)
    {
        reg[0] = 0, reg[1] = 0, reg[2] = 0;
        while (BMI160_ReadRegSeq(0x40, &reg[0], 3) == STATE_PENDING) ;
        if (reg[0] != 0x28 || reg[1] != 0x03 || reg[2] != 0x28)
        {
            TRACE3("t12 FAIL %02X %02X %02X\n", reg[0], reg[1], reg[2]);
            return;
        }
        else
        {
            // continue
        }
    }
    TRACE("t12 PASS\n");
}

// SPI single write test
void t13(void)
{
    uint32_t i;
    uint8_t reg;
    
    for (i = 0; i < 10000; i++)
    {
        reg = 0;
        while (BMI160_WriteReg(0x71, 0x55) == STATE_PENDING) ;
        while (BMI160_ReadReg(0x71, &reg) == STATE_PENDING) ;
        if (reg != 0x55)
        {
            TRACE1("t13 FAIL %02X\n", reg);
            return;
        }
        else
        {
            // continue
        }
    }
    TRACE("t13 PASS\n");
}

// SPI sequencial write test
void t14(void)
{
    uint32_t i;
    uint8_t reg[3];
    
    while (BMI160_SetAccelNormal() == STATE_PENDING) ; // set normal power mode
    for (i = 0; i < 10000; i++)
    {
        reg[0] = 0x55, reg[1] = 0xAA, reg[2] = 0xCC;
        while (BMI160_WriteRegSeq(0x71, &reg[0], 3) == STATE_PENDING) ;
        reg[0] = 0, reg[1] = 0, reg[2] = 0;
        while (BMI160_ReadRegSeq(0x71, &reg[0], 3) == STATE_PENDING) ;
        if (reg[0] != 0x55 || reg[1] != 0xAA || reg[2] != 0xCC)
        {
            TRACE3("t14 FAIL %02X  %02X %02X\n", reg[0], reg[1], reg[2]);
            return;
        }
        else
        {
            // continue
        }
    }
    TRACE("t14 PASS\n");
}

// VCP test
void t2(void)
{
    uint32_t i;
    uint8_t buf[256];
    
    for (i = 0; i < 256; i++)
    {
        buf[i] = i;
    }
    for (i = 0; i < 1000; i++)
    {
        while(CDC_Transmit_FS(&buf[0], 256) == USBD_BUSY) ;
    }
    TRACE("t2 DONE\n");
}

// UART passthrough test
void t3(void)
{
    uint32_t i, j;
    void UART5_TX_enque(uint8_t ch);
    
    for (i = 0; i < 1000; i++)
    {
        for (j = 0; j < 256; j++)
        {
            UART5_TX_enque((uint8_t)j);
        }
    }
    TRACE("t3 DONE\n");
}

// motor speed test
void t4(void)
{
    extern struct _LDS_PID LDS_PID;
    float speed;
    float duty;
    uint32_t PacketCnt; // measurement packet count
    uint32_t SpeedSum;
    float ErrorMin;
    float ErrorMax;
    
    ErrorMin = FLT_MAX;
    ErrorMax = FLT_MIN;
    speed = 0.0f;
    SysTimer = 2000 + 30000;   // 2 + 30 s
    SpeedSum = 0;
    PacketCnt = 0;
    UART5_RX_SetEmpty();
    
    do
    {
        if (lds_get_packet() == 0)
        {// ok
            if (lds_buf[0] == 0xFA)
            {// measurement packet
                if (lds_buf[1] >= 0xA0 && lds_buf[1] <= 0xF9)
                {// valid index
                    SpeedSum += *(uint16_t *)&lds_buf[2];
                    PacketCnt++;
                }
                else
                {
                    TRACE("index FAIL!\n");
                    TRACE_ARRAY(lds_buf, 22);
                }
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
                // keep previou value
            }
            duty = LDS_PID_Update(speed);
            LL_TIM_OC_SetCompareCH3(TIM8, (LL_TIM_GetAutoReload(TIM8) + 1) * duty);
            SpeedCtrlFlag = 0;
            SpeedSum = 0;
            PacketCnt = 0;
        
            if (SysTimer < 30000)
            {
                TRACE2("speed = %10f error = %10f\n", speed, LDS_PID.PreError);
                if (LDS_PID.PreError < ErrorMin)
                {
                    ErrorMin = LDS_PID.PreError;
                }
                if (LDS_PID.PreError > ErrorMax)
                {
                    ErrorMax = LDS_PID.PreError;
                }
            }
        }
    } while (SysTimer != 0);
    
    if (ErrorMin <= -1.0f || ErrorMax >= 1.0f)
    {
        TRACE("t4 FAIL");
    }
    else
    {
        TRACE("t4 PASS");
    }
    TRACE2(" ErrorMin = %10f ErrorMax = %10f\n", ErrorMin, ErrorMax);
    
    LL_TIM_OC_SetCompareCH3(TIM8, 0); // stop
    SysTimer = 3000;
    while (SysTimer != 0) ;
}

// test entry
void t(void)
{
    TRACE("start t11 ...\n");
    t11();
    TRACE("start t12 ...\n");
    t12();
    TRACE("start t13 ...\n");
    t13();
    TRACE("start t14 ...\n");
    t14();
    TRACE("start t2 ...\n");
    t2();
    TRACE("start t3 ...\n");
    t3();
    TRACE("start t4 ...\n");
    t4();
    TRACE("END\n");
}
