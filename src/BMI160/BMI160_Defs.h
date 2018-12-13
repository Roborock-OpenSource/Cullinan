
#ifndef BMI160_DEFS_H
#define BMI160_DEFS_H



#define BMI160_SPI_RD_MASK               0x80
#define BMI160_SPI_WR_MASK               0x7F

// SPI timing, refer to Table 27 of datasheet
#define T_IDLE_WACC_NM          2   // tIDLE_wacc_nm, 2 us
#define T_IDLE_WACC_SUM         450 // tIDLE_wacc_sum, 450 us



/* Delay in ms settings */
#define BMI160_ACCEL_DELAY_MS           5
#define BMI160_GYRO_DELAY_MS            81
#define BMI160_GYRO_F2N_DELAY_MS        (10 + 1)
#define BMI160_ONE_MS_DELAY             1
#define BMI160_AUX_COM_DELAY            10
#define BMI160_GYRO_SELF_TEST_DELAY     20
#define BMI160_ACCEL_SELF_TEST_DELAY    50

/** BMI160 Register map */
#define BMI160_CHIP_ID_ADDR             0x00
#define BMI160_ERROR_REG_ADDR           0x02
#define BMI160_PMU_STATUS_ADDR          0x03
#define BMI160_AUX_DATA_ADDR            0x04
#define BMI160_GYRO_DATA_ADDR           0x0C
#define BMI160_ACCEL_DATA_ADDR          0x12
#define BMI160_STATUS_ADDR              0x1B
#define BMI160_INT_STATUS_ADDR          0x1C
#define BMI160_FIFO_LENGTH_ADDR         0x22
#define BMI160_FIFO_DATA_ADDR           0x24
#define BMI160_ACCEL_CONFIG_ADDR        0x40
#define BMI160_ACCEL_RANGE_ADDR         0x41
#define BMI160_GYRO_CONFIG_ADDR         0x42
#define BMI160_GYRO_RANGE_ADDR          0x43
#define BMI160_AUX_ODR_ADDR             0x44
#define BMI160_FIFO_DOWN_ADDR           0x45
#define BMI160_FIFO_CONFIG_0_ADDR       0x46
#define BMI160_FIFO_CONFIG_1_ADDR       0x47
#define BMI160_AUX_IF_0_ADDR            0x4B
#define BMI160_AUX_IF_1_ADDR            0x4C
#define BMI160_AUX_IF_2_ADDR            0x4D
#define BMI160_AUX_IF_3_ADDR            0x4E
#define BMI160_AUX_IF_4_ADDR            0x4F
#define BMI160_INT_ENABLE_0_ADDR        0x50
#define BMI160_INT_ENABLE_1_ADDR        0x51
#define BMI160_INT_ENABLE_2_ADDR        0x52
#define BMI160_INT_OUT_CTRL_ADDR        0x53
#define BMI160_INT_LATCH_ADDR           0x54
#define BMI160_INT_MAP_0_ADDR           0x55
#define BMI160_INT_MAP_1_ADDR           0x56
#define BMI160_INT_MAP_2_ADDR           0x57
#define BMI160_INT_DATA_0_ADDR          0x58
#define BMI160_INT_DATA_1_ADDR          0x59
#define BMI160_INT_LOWHIGH_0_ADDR       0x5A
#define BMI160_INT_LOWHIGH_1_ADDR       0x5B
#define BMI160_INT_LOWHIGH_2_ADDR       0x5C
#define BMI160_INT_LOWHIGH_3_ADDR       0x5D
#define BMI160_INT_LOWHIGH_4_ADDR       0x5E
#define BMI160_INT_MOTION_0_ADDR        0x5F
#define BMI160_INT_MOTION_1_ADDR        0x60
#define BMI160_INT_MOTION_2_ADDR        0x61
#define BMI160_INT_MOTION_3_ADDR        0x62
#define BMI160_INT_TAP_0_ADDR           0x63
#define BMI160_INT_TAP_1_ADDR           0x64
#define BMI160_INT_ORIENT_0_ADDR        0x65
#define BMI160_INT_ORIENT_1_ADDR        0x66
#define BMI160_INT_FLAT_0_ADDR          0x67
#define BMI160_INT_FLAT_1_ADDR          0x68
#define BMI160_FOC_CONF_ADDR            0x69
#define BMI160_CONF_ADDR                0x6A
#define BMI160_IF_CONF_ADDR             0x6B
#define BMI160_SELF_TEST_ADDR           0x6D
#define BMI160_OFFSET_ADDR              0x71
#define BMI160_OFFSET_CONF_ADDR         0x77
#define BMI160_INT_STEP_CNT_0_ADDR      0x78
#define BMI160_INT_STEP_CONFIG_0_ADDR   0x7A
#define BMI160_INT_STEP_CONFIG_1_ADDR   0x7B
#define BMI160_COMMAND_REG_ADDR         0x7E
#define BMI160_SPI_COMM_TEST_ADDR       0x7F




#endif /* BMI160_DEFS_H */
/* end of BMI160_Defs.h */
