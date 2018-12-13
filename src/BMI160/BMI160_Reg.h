
#ifndef BMI160_REG_H
#define BMI160_REG_H

#include "config.h"

#pragma anon_unions

struct _BMI160_REG
{
    /** BMI160 unique chip identifier */
    enum
    {
        BMI160_CHIP_ID = 0xD1
    } CHIP_ID;                          // 0x00
    uint8_t RSVD1;                      // 0x01
    union
    {
        uint8_t all;
        struct
        {
            struct
            {
                uint8_t fatal_err :1;
            };
        };
    } ERR_REG;                          // 0x02
    union
    {
        uint8_t all;
        enum
        {
            BMI160_MAG_PMU_SUSPEND      = 0x00,
            BMI160_MAG_PMU_NORMAL       = 0x01,
            BMI160_MAG_PMU_LOW_POWER    = 0x02,
        } mag_pmu_status : 2;
        enum
        {
            BMI160_GYR_PMU_SUSPEND      = 0x00,
            BMI160_GYR_PMU_NORMAL       = 0x01,
            BMI160_GYR_PMU_FAST_STARTUP = 0x03,
        } gyr_pmu_status : 2;
        enum
        {
            BMI160_ACC_PMU_SUSPEND      = 0x00,
            BMI160_ACC_PMU_NORMAL       = 0x01,
            BMI160_ACC_PMU_LOW_POWER    = 0x02,
        } acc_pmu_status : 2;
        uint8_t             : 2;
    } PMU_STATUS;                       // 0x03
    struct
    {
        uint8_t mag_x_7_0;
        uint8_t mag_x_15_8;
        uint8_t mag_y_7_0;
        uint8_t mag_y_15_8;
        uint8_t mag_z_7_0;
        uint8_t mag_z_15_8;
        uint8_t rhall_7_0;
        uint8_t rhall_15_8;
        uint8_t gyr_x_7_0;
        uint8_t gyr_x_15_8;
        uint8_t gyr_y_7_0;
        uint8_t gyr_y_15_8;
        uint8_t gyr_z_7_0;
        uint8_t gyr_z_15_8;
        uint8_t acc_x_7_0;
        uint8_t acc_x_15_8;
        uint8_t acc_y_7_0;
        uint8_t acc_y_15_8;
        uint8_t acc_z_7_0;
        uint8_t acc_z_15_8;
    } DATA;                             // 0x04 - 0x17
    struct
    {
        uint8_t sensor_time_7_0;
        uint8_t sensor_time_15_8;
        uint8_t sensor_time_23_16;
    } SENSORTIME;                       // 0x18 - 0x1A
    struct
    {
        uint8_t                     : 1;
        uint8_t gyr_self_test_ok    : 1;
        uint8_t mag_man_op          : 1;
        uint8_t foc_rdy             : 1;
        uint8_t nvm_rdy             : 1;
        uint8_t drdy_mag            : 1;
        uint8_t drdy_gyr            : 1;
        uint8_t drdy_acc            : 1;
    } STATUS;                           // 0x1B
    union
    {
        uint8_t all[4];
        struct
        {
            uint8_t  step_int                   : 1;
        };
    } INT_STATUS;                       // 0x1C - 0x1F
    struct
    {
        uint8_t temperature_7_0;
        uint8_t temperature_15_8;
    } TEMPERATURE;                      // 0x20 - 0x21
    struct
    {
        uint8_t fifo_length_7_0     : 8;
        uint8_t fifo_length_10_8    : 3;
    } FIFO_LENGTH;                      // 0x22 - 0x23
    uint8_t FIFO_DATA;                  // 0x24
    uint8_t rsvd253f[27];               // 0x25 - 0x3F
    union
    {
        uint8_t all;
        struct
        {
            /* Accel Output data rate */
            enum
            {
                BMI160_ACCEL_ODR_RESERVED        = 0x00,
                BMI160_ACCEL_ODR_0_78HZ          = 0x01,
                BMI160_ACCEL_ODR_1_56HZ          = 0x02,
                BMI160_ACCEL_ODR_3_12HZ          = 0x03,
                BMI160_ACCEL_ODR_6_25HZ          = 0x04,
                BMI160_ACCEL_ODR_12_5HZ          = 0x05,
                BMI160_ACCEL_ODR_25HZ            = 0x06,
                BMI160_ACCEL_ODR_50HZ            = 0x07,
                BMI160_ACCEL_ODR_100HZ           = 0x08,
                BMI160_ACCEL_ODR_200HZ           = 0x09,
                BMI160_ACCEL_ODR_400HZ           = 0x0A,
                BMI160_ACCEL_ODR_800HZ           = 0x0B,
                BMI160_ACCEL_ODR_1600HZ          = 0x0C,
                BMI160_ACCEL_ODR_RESERVED0       = 0x0D,
                BMI160_ACCEL_ODR_RESERVED1       = 0x0E,
                BMI160_ACCEL_ODR_RESERVED2       = 0x0F,
            } acc_odr :4;
            /* Accel Bandwidth */
            enum
            {
                BMI160_ACCEL_BW_OSR4_AVG1        = 0x00,
                BMI160_ACCEL_BW_OSR2_AVG2        = 0x01,
                BMI160_ACCEL_BW_NORMAL_AVG4      = 0x02,
                BMI160_ACCEL_BW_RES_AVG8         = 0x03,
                BMI160_ACCEL_BW_RES_AVG16        = 0x04,
                BMI160_ACCEL_BW_RES_AVG32        = 0x05,
                BMI160_ACCEL_BW_RES_AVG64        = 0x06,
                BMI160_ACCEL_BW_RES_AVG128       = 0x07,
            } acc_bwp : 3;
            uint8_t acc_us  :1;
        };
    } ACC_CONF;                         // 0x40
    union
    {
        uint8_t all;
        struct
        {
            /* Accel Range */
            enum
            {
                BMI160_ACCEL_RANGE_2G            = 0x03,
                BMI160_ACCEL_RANGE_4G            = 0x05,
                BMI160_ACCEL_RANGE_8G            = 0x08,
                BMI160_ACCEL_RANGE_16G           = 0x0C,
            } acc_range : 4;
            uint8_t             :4;
        };
    } ACC_RANGE;                        // 0x41
    union
    {
        uint8_t all;
        struct
        {
            /* Gyro Output data rate */
            enum
            {
                BMI160_GYRO_ODR_RESERVED    = 0x00,
                BMI160_GYRO_ODR_25HZ        = 0x06,
                BMI160_GYRO_ODR_50HZ        = 0x07,
                BMI160_GYRO_ODR_100HZ       = 0x08,
                BMI160_GYRO_ODR_200HZ       = 0x09,
                BMI160_GYRO_ODR_400HZ       = 0x0A,
                BMI160_GYRO_ODR_800HZ       = 0x0B,
                BMI160_GYRO_ODR_1600HZ      = 0x0C,
                BMI160_GYRO_ODR_3200HZ      = 0x0D,
            } gyr_odr : 4;
            /* Gyro Bandwidth */
            enum
            {
                BMI160_GYRO_BW_OSR4_MODE    = 0x00,
                BMI160_GYRO_BW_OSR2_MODE    = 0x01,
                BMI160_GYRO_BW_NORMAL_MODE  = 0x02,
            } gyr_bwp : 2;
            uint8_t         : 2;
        };
    } GYR_CONF;                         // 0x42
    union
    {
        uint8_t all;
        struct
        {
            /* Gyro Range */
            enum
            {
                BMI160_GYRO_RANGE_2000_DPS  = 0x00,
                BMI160_GYRO_RANGE_1000_DPS  = 0x01,
                BMI160_GYRO_RANGE_500_DPS   = 0x02,
                BMI160_GYRO_RANGE_250_DPS   = 0x03,
                BMI160_GYRO_RANGE_125_DPS   = 0x04,
                BMI160_GYRO_RANGE_MAX       = 0x04
            } gyr_range   : 3;
            uint8_t             : 5;
        };
    } GYR_RANGE;                        // 0x43
    union
    {
        uint8_t all;
        struct
        {
            uint8_t tbd       : 1;
            uint8_t   : 1;
            uint8_t    : 1;
            uint8_t             : 5;
        };
    } MAG_CONF;                         // 0x44
    union
    {
        uint8_t all;
        struct
        {
            uint8_t tbd       : 1;
            uint8_t   : 1;
            uint8_t    : 1;
            uint8_t             : 5;
        };
    } FIFO_DOWNS;                       // 0x45
    union
    {
        uint8_t all[2];
        struct
        {
            uint8_t tbd       : 1;
            uint8_t   : 1;
            uint8_t    : 1;
            uint8_t             : 5;
        };
    } FIFO_CONFIG;                      // 0x46 - 0x47
    uint8_t rsvd484a[3];                // 0x48 - 0x4A
    union
    {
        uint8_t all[5];
        struct
        {
            uint8_t tbd       : 1;
            uint8_t   : 1;
            uint8_t    : 1;
            uint8_t             : 5;
        };
    } MAG_IF;                           // 0x4B - 0x4F
    union
    {
        uint8_t all[3];
        struct
        {
            uint8_t int_anymo_x_en  : 1;
            uint8_t int_anymo_y_en  : 1;
            uint8_t int_anymo_z_en  : 1;
            uint8_t                 : 1;
            uint8_t int_d_tap_en    : 1;
            uint8_t int_s_tap_en    : 1;
            uint8_t int_orient_en   : 1;
            uint8_t int_flat_en     : 1;
            
            uint8_t int_highx_en    : 1;
            uint8_t int_highy_en    : 1;
            uint8_t int_highz_en    : 1;
            uint8_t int_low_en      : 1;
            uint8_t int_drdy_en     : 1;
            uint8_t int_ffull_en    : 1;
            uint8_t int_fwm_en      : 1;
            uint8_t                 : 1;
            
            uint8_t int_nomox_en    : 1;
            uint8_t int_nomoy_en    : 1;
            uint8_t int_nomoz_en    : 1;
            uint8_t int_step_det_en : 1;
            uint8_t                 : 4;
        };
    } INT_EN;                           // 0x50 - 0x52
    union
    {
        uint8_t all;
        struct
        {
            uint8_t int1_edge_crtl  : 1;
            uint8_t int1_lvl        : 1;
            uint8_t int1_od         : 1;
            uint8_t int1_output_en  : 1;
            uint8_t int2_edge_crtl  : 1;
            uint8_t int2_lvl        : 1;
            uint8_t int2_od         : 1;
            uint8_t int2_output_en  : 1;
        };
    } INT_OUT_CTRL;                     // 0x53
    union
    {
        uint8_t all;
        struct
        {
            uint8_t tbd       : 1;
            uint8_t   : 1;
            uint8_t    : 1;
            uint8_t             : 5;
        };
    } INT_LATCH;                        // 0x54
    union
    {
        uint8_t all[3];
        struct
        {
            uint8_t int1_lowg_step      : 1;
            uint8_t int1_highg          : 1;
            uint8_t int1_anymotion      : 1;
            uint8_t int1_nomotion       : 1;
            uint8_t int1_d_tap          : 1;
            uint8_t int1_s_tap          : 1;
            uint8_t int1_orient         : 1;
            uint8_t int1_flat           : 1;
            
            uint8_t int2_pmu_trig       : 1;
            uint8_t int2_ffull          : 1;
            uint8_t int2_fwm            : 1;
            uint8_t int2_drdy           : 1;
            uint8_t int1_pmu_trig       : 1;
            uint8_t int1_ffull          : 1;
            uint8_t int1_fwm            : 1;
            uint8_t int1_drdy           : 1;
            
            uint8_t int2_lowg_step      : 1;
            uint8_t int2_highg          : 1;
            uint8_t int2_anymotion      : 1;
            uint8_t int2_nomotion       : 1;
            uint8_t int2_d_tap          : 1;
            uint8_t int2_s_tap          : 1;
            uint8_t int2_orient         : 1;
            uint8_t int2_flat           : 1;
        };
    } INT_MAP;                          // 0x55 - 0x57
    union
    {
        uint8_t all[2];
        struct
        {
            uint8_t tbd       : 1;
            uint8_t   : 1;
            uint8_t    : 1;
            uint8_t             : 5;
        };
    } INT_DATA;                         // 0x58- 0x59
    union
    {
        uint8_t all[5];
        struct
        {
            uint8_t tbd       : 1;
            uint8_t   : 1;
            uint8_t    : 1;
            uint8_t             : 5;
        };
    } INT_LOWHIGH;                      // 0x5A- 0x5E
    union
    {
        uint8_t all[4];
        struct
        {
            uint8_t tbd       : 1;
            uint8_t   : 1;
            uint8_t    : 1;
            uint8_t             : 5;
        };
    } INT_MOTION;                       // 0x5F- 0x62
    
    union
    {
        uint8_t all[2];
        struct
        {
            uint8_t  tbd      : 1;
            uint8_t   : 1;
            uint8_t    : 1;
            uint8_t             : 5;
        };
    } INT_TAP;                          // 0x63- 0x64
    union
    {
        uint8_t all[2];
        struct
        {
            uint8_t  tbd      : 1;
            uint8_t   : 1;
            uint8_t    : 1;
            uint8_t             : 5;
        };
    } INT_ORIENT;                       // 0x65- 0x66
    union
    {
        uint8_t all[2];
        struct
        {
            uint8_t  tbd      : 1;
            uint8_t   : 1;
            uint8_t    : 1;
            uint8_t             : 5;
        };
    } INT_FLAT;                         // 0x67- 0x68
    union
    {
        uint8_t all;
        struct
        {
            uint8_t  tbd      : 1;
            uint8_t   : 1;
            uint8_t    : 1;
            uint8_t             : 5;
        };
    } FOC_CONF;                         // 0x69
    union
    {
        uint8_t all;
        struct
        {
            uint8_t  tbd      : 1;
            uint8_t   : 1;
            uint8_t    : 1;
            uint8_t             : 5;
        };
    } CONF;                             // 0x6A
    union
    {
        uint8_t all;
        struct
        {
            uint8_t tbd       : 1;
            uint8_t   : 1;
            uint8_t    : 1;
            uint8_t             : 5;
        };
    } IF_CONF;                          // 0x6B
    union
    {
        uint8_t all;
        struct
        {
            uint8_t  tbd      : 1;
            uint8_t   : 1;
            uint8_t    : 1;
            uint8_t             : 5;
        };
    } PMU_TRIGGER;                      // 0x6C
    union
    {
        uint8_t all;
        struct
        {
            uint8_t tbd       : 1;
            uint8_t   : 1;
            uint8_t    : 1;
            uint8_t             : 5;
        };
    } SELF_TEST;                        // 0x6D
    uint8_t rsvd6e6f[2];                // 0x6E - 0x6F
    union
    {
        uint8_t all;
        struct
        {
            uint8_t spi_en      : 1;
            uint8_t i2c_wdt_sel : 1;
            uint8_t i2c_wdt_en  : 1;
            uint8_t             : 5;
        };
    } NV_CONF;                          // 0x70
    struct
    {
        uint8_t all[7];
        
    } OFFSET;                           // 0x71 - 0x77
    struct
    {
        uint8_t all[2];
        
    } STEP_CNT;                        // 0x78 - 0x79
    union
    {
        uint8_t all[2];
        
    } STEP_CONF;                        // 0x7A - 0x7B
    uint8_t rsvd7c7d[2];                // 0x7C - 0x7D
    enum
    {
        /** Soft reset command */
        BMI160_SOFT_RESET_CMD           = 0xB6,
            #define BMI160_SOFT_RESET_DELAY_MS       15
        /** Start FOC command */
        BMI160_START_FOC_CMD            = 0x03,
        /* Accel power mode */
        BMI160_ACCEL_SUSPEND_MODE       = 0x10,
        BMI160_ACCEL_NORMAL_MODE        = 0x11,
        BMI160_ACCEL_LOWPOWER_MODE      = 0x12,
        /* Gyro power mode */
        BMI160_GYRO_SUSPEND_MODE        = 0x14,
        BMI160_GYRO_NORMAL_MODE         = 0x15,
        BMI160_GYRO_FASTSTARTUP_MODE    = 0x17,
        /** NVM backup enabling command */
        BMI160_NVM_BACKUP_EN            = 0xA0
    } CMD;                              // 0x7E
    uint8_t SPI_COMM_TEST;              // 0x7F
};


#define REG(r) OFFSET(struct _BMI160_REG, r)

int32_t BMI160_ReadReg(uint8_t RegAddr, uint8_t *RegData);
int32_t BMI160_ReadRegSeq(uint8_t RegAddr, uint8_t *RegData, uint8_t len);

int32_t BMI160_WriteReg(uint8_t RegAddr, uint8_t RegData);
int32_t BMI160_ReadBlock(uint8_t RegAddr, uint8_t *RegDataBuf, uint8_t n);
int32_t BMI160_WriteRegSeq(uint8_t RegAddr, uint8_t *RegData, uint8_t len);

#endif /* BMI160_REG_H */
/* end of BMI160_Reg.h */
